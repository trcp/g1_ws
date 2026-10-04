#!/usr/bin/env python3
# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

"""マイク録音データの分析，定常ノイズ抑制，RMS 適応ゲイン調整モジュール．"""

from typing import Optional, Tuple
from dataclasses import dataclass
import math

import numpy as np


FULL_SCALE_INT16 = 32768.0
CLIPPING_THRESHOLD_INT16 = 32760


@dataclass(frozen=True)
class AudioMetrics:
    """PCM音声データの統計情報です．

    Parameters
    ----------
    sample_count, peak, peak_dbfs, rms, rms_dbfs : float
        音声振幅の統計値です．

    Methods
    -------
    なし
    """

    sample_count: int
    peak: float
    peak_dbfs: float
    rms: float
    rms_dbfs: float
    mean_abs: float
    p95_abs: float
    p99_abs: float
    clip_count: int
    clip_fraction: float
    crest_factor_db: float


@dataclass(frozen=True)
class AdaptiveGainConfig:
    """適応型RMSゲインの設定パラメータです．

    Parameters
    ----------
    target_speech_rms_dbfs, peak_ceiling_dbfs : float
        目標RMSとピーク上限です．

    Methods
    -------
    なし
    """

    target_speech_rms_dbfs: float = -16.0  # 人間の耳と Whisper に適した明瞭な音量
    peak_ceiling_dbfs: float = -3.0        # デジタル音割れを防ぐ安全上限（約 23,200）
    max_gain_db: float = 14.0              # 最大増幅倍率上限（約 5.0x）
    min_gain_db: float = -12.0             # 過大入力時の減衰下限（約 0.25x）
    speech_snr_margin_db: float = 6.0
    absolute_speech_floor_dbfs: float = -45.0
    frame_ms: float = 20.0
    hop_ms: float = 10.0



@dataclass(frozen=True)
class NoiseSuppressionConfig:
    """定常ノイズ抑制の設定パラメータです．

    Parameters
    ----------
    n_fft, hop_length : int
        STFTのFFT長とホップ長です．
    alpha, min_gain : float
        抑制強度と最小ゲインです．

    Methods
    -------
    なし
    """

    n_fft: int = 512
    hop_length: int = 256
    alpha: float = 1.5
    min_gain: float = 0.20  # 最大減衰量を約 14 dB に制限し歪みを防止


@dataclass(frozen=True)
class AudioProcessingResult:
    """マイク音声処理結果と音響メトリクスです．

    Parameters
    ----------
    processed_samples : numpy.ndarray
        処理後のPCMサンプルです．

    Methods
    -------
    なし
    """

    processed_samples: np.ndarray
    raw_metrics: AudioMetrics
    denoised_metrics: Optional[AudioMetrics]
    processed_metrics: AudioMetrics
    applied_gain: float
    applied_gain_db: float
    noise_metrics: Optional[AudioMetrics]
    noise_reduction_db: float
    speech_detected: bool


def pcm_to_dbfs(linear_val: float) -> float:
    """線形振幅をフルスケール dBFS へ変換します．"""
    if linear_val <= 1e-9:
        return -120.0
    return 20.0 * math.log10(max(1e-9, linear_val / FULL_SCALE_INT16))


def analyze_pcm16(
    samples: np.ndarray,
    clip_threshold: int = CLIPPING_THRESHOLD_INT16,
) -> AudioMetrics:
    """int16 PCM 配列の音響統計およびクリッピング指標を算出します．"""
    count = int(samples.size)
    if count == 0:
        return AudioMetrics(
            sample_count=0,
            peak=0.0,
            peak_dbfs=-120.0,
            rms=0.0,
            rms_dbfs=-120.0,
            mean_abs=0.0,
            p95_abs=0.0,
            p99_abs=0.0,
            clip_count=0,
            clip_fraction=0.0,
            crest_factor_db=0.0,
        )

    f64 = samples.astype(np.float64, copy=False)
    abs_vals = np.abs(f64)
    peak = float(np.max(abs_vals))
    mean_abs = float(np.mean(abs_vals))
    rms = float(np.sqrt(np.mean(f64 ** 2)))
    p95 = float(np.percentile(abs_vals, 95))
    p99 = float(np.percentile(abs_vals, 99))
    clip_count = int(np.count_nonzero(abs_vals >= clip_threshold))
    clip_fraction = float(clip_count / count)

    peak_dbfs = pcm_to_dbfs(peak)
    rms_dbfs = pcm_to_dbfs(rms)

    if rms > 1e-9 and peak > 1e-9:
        crest_factor_db = 20.0 * math.log10(peak / rms)
    else:
        crest_factor_db = 0.0

    return AudioMetrics(
        sample_count=count,
        peak=peak,
        peak_dbfs=peak_dbfs,
        rms=rms,
        rms_dbfs=rms_dbfs,
        mean_abs=mean_abs,
        p95_abs=p95,
        p99_abs=p99,
        clip_count=clip_count,
        clip_fraction=clip_fraction,
        crest_factor_db=crest_factor_db,
    )


def __stft(
    x: np.ndarray,
    n_fft: int = 512,
    hop_length: int = 256,
) -> Tuple[np.ndarray, np.ndarray]:
    """Centered STFT を行い，完全な再構成を保証します．"""
    pad_amount = n_fft // 2
    x_padded = np.pad(x, (pad_amount, pad_amount), mode='reflect')
    window = np.hanning(n_fft)

    num_frames = 1 + (len(x_padded) - n_fft) // hop_length
    frames = np.lib.stride_tricks.as_strided(
        x_padded,
        shape=(num_frames, n_fft),
        strides=(x_padded.strides[0] * hop_length, x_padded.strides[0]),
        writeable=False,
    )
    windowed = frames * window
    spec = np.fft.rfft(windowed, n=n_fft)
    return spec, window


def __istft(
    spec: np.ndarray,
    window: np.ndarray,
    hop_length: int,
    target_length: int,
) -> np.ndarray:
    """Centered ISTFT により元の時間波形を完全再構成します．"""
    num_frames, num_freqs = spec.shape
    n_fft = (num_freqs - 1) * 2
    pad_amount = n_fft // 2
    out_len = (num_frames - 1) * hop_length + n_fft
    out = np.zeros(out_len, dtype=np.float64)
    win_norm = np.zeros(out_len, dtype=np.float64)
    win_sq = window ** 2

    time_frames = np.fft.irfft(spec, n=n_fft)
    for i in range(num_frames):
        start = i * hop_length
        end = start + n_fft
        out[start:end] += time_frames[i] * window
        win_norm[start:end] += win_sq

    nonzero_mask = win_norm > 1e-12
    out[nonzero_mask] /= win_norm[nonzero_mask]

    # パディング分を取り除き元の長さを復元
    unpadded = out[pad_amount:pad_amount + target_length]
    if len(unpadded) < target_length:
        return np.pad(
            unpadded, (0, target_length - len(unpadded)), mode='constant'
        )
    return unpadded


def estimate_stationary_noise(
    noise_samples: np.ndarray,
    n_fft: int = 512,
    hop_length: int = 256,
) -> np.ndarray:
    """無音・ファン音区間からパワースペクトルの中央値によりノイズ PSD を推定します．"""
    if noise_samples.size < n_fft:
        return np.zeros(n_fft // 2 + 1, dtype=np.float64)

    f64 = noise_samples.astype(np.float64, copy=False)
    f64 = f64 - np.mean(f64)
    spec, _ = __stft(f64, n_fft=n_fft, hop_length=hop_length)
    power = np.abs(spec) ** 2
    noise_psd = np.median(power, axis=0)
    return np.maximum(noise_psd, 1e-12)


def suppress_stationary_noise(
    samples: np.ndarray,
    noise_psd: np.ndarray,
    config: Optional[NoiseSuppressionConfig] = None,
) -> np.ndarray:
    """定常ノイズ PSD を用いて Wiener 型ソフトマスクによるノイズ低減を行います．"""
    cfg = config or NoiseSuppressionConfig()
    target_length = len(samples)
    if target_length < cfg.n_fft or noise_psd.size != (cfg.n_fft // 2 + 1):
        return samples.copy()

    f64 = samples.astype(np.float64, copy=False)
    dc = float(np.mean(f64))
    f64 = f64 - dc

    spec, window = __stft(f64, n_fft=cfg.n_fft, hop_length=cfg.hop_length)
    power = np.abs(spec) ** 2

    speech_power = np.maximum(power - cfg.alpha * noise_psd, 0.0)
    mask = speech_power / (speech_power + noise_psd + 1e-12)
    mask = np.clip(mask, cfg.min_gain, 1.0)

    clean_spec = spec * mask
    out = __istft(
        clean_spec,
        window=window,
        hop_length=cfg.hop_length,
        target_length=target_length,
    )
    return out + dc


def apply_adaptive_gain(
    samples: np.ndarray,
    noise_rms_dbfs: Optional[float] = None,
    config: Optional[AdaptiveGainConfig] = None,
    sample_rate: int = 16000,
) -> Tuple[np.ndarray, float, float, bool]:
    """Active speech RMS とピーク安全制約に基づき，全体に安定した 1 つのゲインを適用します．

    Returns
    -------
    Tuple[np.ndarray, float, float, bool]
        (処理済み int16 サンプル, 適用ゲイン倍率, 適用ゲイン dB, 発話検出フラグ)
    """
    cfg = config or AdaptiveGainConfig()
    if samples.size == 0:
        return np.empty(0, dtype=np.int16), 1.0, 0.0, False

    f64 = samples.astype(np.float64, copy=False)
    dc = float(np.mean(f64))
    f64 = f64 - dc

    frame_len = int(sample_rate * (cfg.frame_ms / 1000.0))
    hop_len = int(sample_rate * (cfg.hop_ms / 1000.0))

    if len(f64) < frame_len:
        raw_rms = float(np.sqrt(np.mean(f64 ** 2)))
        active_speech_rms_dbfs = pcm_to_dbfs(raw_rms)
        speech_detected = active_speech_rms_dbfs >= cfg.absolute_speech_floor_dbfs
    else:
        num_frames = 1 + (len(f64) - frame_len) // hop_len
        frames = np.lib.stride_tricks.as_strided(
            f64,
            shape=(num_frames, frame_len),
            strides=(f64.strides[0] * hop_len, f64.strides[0]),
            writeable=False,
        )
        frame_rms = np.sqrt(np.mean(frames ** 2, axis=1))
        frame_rms_dbfs = np.array([pcm_to_dbfs(r) for r in frame_rms])

        if noise_rms_dbfs is not None:
            threshold_dbfs = max(
                noise_rms_dbfs + cfg.speech_snr_margin_db,
                cfg.absolute_speech_floor_dbfs,
            )
        else:
            p20 = float(np.percentile(frame_rms_dbfs, 20))
            p90 = float(np.percentile(frame_rms_dbfs, 90))
            if p90 - p20 < 3.0:
                # 信号全体のレベルがほぼ一定（定常音・連続発話）の場合
                threshold_dbfs = cfg.absolute_speech_floor_dbfs
            else:
                threshold_dbfs = max(
                    p20 + cfg.speech_snr_margin_db,
                    cfg.absolute_speech_floor_dbfs,
                )

        active_indices = np.where(frame_rms_dbfs > threshold_dbfs)[0]
        if len(active_indices) >= 3:
            speech_detected = True
            active_rms_vals = frame_rms[active_indices]
            active_mean_rms = float(np.sqrt(np.mean(active_rms_vals ** 2)))
            active_speech_rms_dbfs = pcm_to_dbfs(active_mean_rms)
        else:
            speech_detected = False
            active_speech_rms_dbfs = pcm_to_dbfs(float(np.sqrt(np.mean(f64 ** 2))))


    if not speech_detected:
        applied_gain_db = min(0.0, cfg.max_gain_db)
    else:
        desired_db = cfg.target_speech_rms_dbfs - active_speech_rms_dbfs
        applied_gain_db = max(cfg.min_gain_db, min(desired_db, cfg.max_gain_db))

    linear_gain = 10.0 ** (applied_gain_db / 20.0)

    abs_f64 = np.abs(f64)
    peak_ceiling_linear = (
        10.0 ** (cfg.peak_ceiling_dbfs / 20.0) * FULL_SCALE_INT16
    )
    # 単発スパイク（息や衝撃音）で声全体が消えるのを防ぐため p99.5 を主指標に採用
    p99_5 = float(np.percentile(abs_f64, 99.5)) if abs_f64.size > 0 else 0.0
    raw_peak = float(np.max(abs_f64)) if abs_f64.size > 0 else 0.0
    representative_peak = max(p99_5, raw_peak * 0.75)

    if (
        representative_peak * linear_gain > peak_ceiling_linear
        and representative_peak > 1e-6
    ):
        safe_linear_gain = peak_ceiling_linear / representative_peak
        linear_gain = min(linear_gain, safe_linear_gain)
        applied_gain_db = 20.0 * math.log10(max(1e-9, linear_gain))

    scaled = f64 * linear_gain
    # knee（ceilingの85%）を超える突出スパイクのみを ceiling へ向けて滑らかに圧縮
    knee = peak_ceiling_linear * 0.85
    abs_scaled = np.abs(scaled)
    over_knee = abs_scaled > knee
    if np.any(over_knee):
        headroom = peak_ceiling_linear - knee
        excess = abs_scaled[over_knee] - knee
        compressed = knee + headroom * np.tanh(excess / max(1.0, headroom))
        scaled[over_knee] = np.sign(scaled[over_knee]) * compressed

    # 厳密に peak_ceiling_linear 以内に収める
    boosted = np.clip(scaled, -peak_ceiling_linear, peak_ceiling_linear)
    return (
        np.round(boosted).astype(np.int16),
        linear_gain,
        applied_gain_db,
        speech_detected,
    )



def condition_mic_audio(
    samples: np.ndarray,
    noise_samples: Optional[np.ndarray] = None,
    gain_config: Optional[AdaptiveGainConfig] = None,
    noise_config: Optional[NoiseSuppressionConfig] = None,
    sample_rate: int = 16000,
) -> AudioProcessingResult:
    """マイク録音データに対する総合的な音響分析・ノイズ抑制・適応ゲイン処理を実施します．"""
    raw_metrics = analyze_pcm16(samples)
    if samples.size == 0:
        empty_i16 = np.empty(0, dtype=np.int16)
        return AudioProcessingResult(
            processed_samples=empty_i16,
            raw_metrics=raw_metrics,
            denoised_metrics=None,
            processed_metrics=raw_metrics,
            applied_gain=1.0,
            applied_gain_db=0.0,
            noise_metrics=None,
            noise_reduction_db=0.0,
            speech_detected=False,
        )

    noise_metrics = None
    noise_psd = None
    noise_rms_dbfs = None

    if noise_samples is not None and noise_samples.size > 0:
        noise_metrics = analyze_pcm16(noise_samples)
        noise_rms_dbfs = noise_metrics.rms_dbfs
        n_cfg = noise_config or NoiseSuppressionConfig()
        noise_psd = estimate_stationary_noise(
            noise_samples,
            n_fft=n_cfg.n_fft,
            hop_length=n_cfg.hop_length,
        )

    if noise_psd is not None:
        denoised_f64 = suppress_stationary_noise(
            samples,
            noise_psd=noise_psd,
            config=noise_config,
        )
        denoised_i16 = np.clip(denoised_f64, -32767.0, 32767.0).astype(np.int16)
        denoised_metrics = analyze_pcm16(denoised_i16)
        target_for_gain = denoised_f64
        noise_reduction_db = max(0.0, raw_metrics.rms_dbfs - denoised_metrics.rms_dbfs)
    else:
        denoised_metrics = None
        target_for_gain = samples
        noise_reduction_db = 0.0

    (
        processed_samples,
        applied_gain,
        applied_gain_db,
        speech_detected,
    ) = apply_adaptive_gain(
        target_for_gain,
        noise_rms_dbfs=noise_rms_dbfs,
        config=gain_config,
        sample_rate=sample_rate,
    )

    processed_metrics = analyze_pcm16(processed_samples)

    return AudioProcessingResult(
        processed_samples=processed_samples,
        raw_metrics=raw_metrics,
        denoised_metrics=denoised_metrics,
        processed_metrics=processed_metrics,
        applied_gain=applied_gain,
        applied_gain_db=applied_gain_db,
        noise_metrics=noise_metrics,
        noise_reduction_db=noise_reduction_db,
        speech_detected=speech_detected,
    )
