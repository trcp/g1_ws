from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Literal

import numpy as np


@dataclass
class BellDetectionConfig:
    sample_rate: int = 16000
    frame_size: int = 512
    hop_size: int = 160
    bell_band_min_hz: float = 800.0
    bell_band_max_hz: float = 4000.0
    total_band_min_hz: float = 80.0
    total_band_max_hz: float = 7000.0
    min_snr_db: float = 10.0
    min_high_energy_ratio: float = 0.30
    min_peak_to_median_db: float = 10.0
    max_spectral_flatness: float = 0.35
    min_note1_duration_s: float = 0.08
    max_note1_duration_s: float = 0.70
    min_note2_duration_s: float = 0.12
    max_note2_duration_s: float = 1.20
    min_gap_s: float = 0.03
    max_gap_s: float = 0.70
    max_freq_std_hz: float = 180.0
    max_freq_std_ratio: float = 0.15
    freq_relation: Literal["loose", "falling", "any"] = "loose"
    max_missing_tonal_frames: int = 3
    cooldown_s: float = 3.0
    noise_history_size: int = 300
    warmup_s: float = 0.3
    min_event_score: float = 0.65


@dataclass
class FrameFeatures:
    time_s: float
    rms_db: float
    snr_db: float
    high_energy_ratio: float
    peak_freq_hz: float
    peak_to_median_db: float
    spectral_flatness: float
    tonal_score: float
    is_tonal: bool


@dataclass
class BellEvent:
    start_time_s: float
    end_time_s: float
    note1_freq_hz: float
    note2_freq_hz: float
    note1_duration_s: float
    note2_duration_s: float
    gap_s: float
    score: float


class BellDetector:
    def __init__(self, config: BellDetectionConfig | None = None):
        self.config = config or BellDetectionConfig()
        self.reset()

    def reset(self) -> None:
        cfg = self.config
        self._buffer = np.zeros(0, dtype=np.float32)
        self._frame_start_sample = 0
        self._window = np.hanning(cfg.frame_size).astype(np.float32)
        self._freqs = np.fft.rfftfreq(cfg.frame_size, d=1.0 / cfg.sample_rate)
        self._bell_mask = (
            (self._freqs >= cfg.bell_band_min_hz)
            & (self._freqs <= cfg.bell_band_max_hz)
        )
        self._total_mask = (
            (self._freqs >= cfg.total_band_min_hz)
            & (self._freqs <= cfg.total_band_max_hz)
        )
        if not np.any(self._bell_mask):
            raise ValueError("bell_band does not contain any FFT bins")
        if not np.any(self._total_mask):
            raise ValueError("total_band does not contain any FFT bins")

        self._noise_history: deque[float] = deque(maxlen=cfg.noise_history_size)
        self._noise_floor_db = -80.0
        self._state = "IDLE"
        self._cooldown_until_s = 0.0
        self._note1: dict | None = None
        self._note2: dict | None = None
        self._missing_tonal_frames = 0
        self._gap_start_s: float | None = None

    @property
    def state(self) -> str:
        return self._state

    @property
    def noise_floor_db(self) -> float:
        return self._noise_floor_db

    def predict(self, audio_chunk: np.ndarray) -> list[BellEvent]:
        x = self._to_float_mono(audio_chunk)
        if x.size == 0:
            return []

        self._buffer = np.concatenate([self._buffer, x])
        events: list[BellEvent] = []
        cfg = self.config

        while self._buffer.size >= cfg.frame_size:
            frame = self._buffer[: cfg.frame_size]
            self._buffer = self._buffer[cfg.hop_size :]
            t = self._frame_start_sample / cfg.sample_rate
            self._frame_start_sample += cfg.hop_size
            event = self._update_state(self._extract_features(frame, t))
            if event is not None:
                events.append(event)

        return events

    def predict_bool(self, audio_chunk: np.ndarray) -> bool:
        return len(self.predict(audio_chunk)) > 0

    def _to_float_mono(self, audio_chunk: np.ndarray) -> np.ndarray:
        x = np.asarray(audio_chunk)
        if x.ndim == 2:
            x = x.mean(axis=1)
        if x.dtype == np.int16:
            x = x.astype(np.float32) / 32768.0
        else:
            x = x.astype(np.float32)
        return np.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0)

    def _extract_features(self, frame: np.ndarray, time_s: float) -> FrameFeatures:
        cfg = self.config
        eps = 1e-12
        frame = frame.astype(np.float32)
        frame = frame - float(np.mean(frame))
        rms = float(np.sqrt(np.mean(frame * frame) + eps))
        rms_db = 20.0 * np.log10(rms + eps)
        snr_db = rms_db - self._noise_floor_db

        spectrum = np.fft.rfft(frame * self._window)
        power = np.abs(spectrum) ** 2
        total_power = float(np.sum(power[self._total_mask]) + eps)
        bell_power = power[self._bell_mask]
        high_power = float(np.sum(bell_power) + eps)
        high_energy_ratio = high_power / total_power

        peak_idx_local = int(np.argmax(bell_power))
        bell_freqs = self._freqs[self._bell_mask]
        peak_freq_hz = float(bell_freqs[peak_idx_local])
        peak_power = float(bell_power[peak_idx_local] + eps)
        median_power = float(np.median(bell_power) + eps)
        peak_to_median_db = 10.0 * np.log10(peak_power / median_power)
        spectral_flatness = float(
            np.exp(np.mean(np.log(bell_power + eps))) / (np.mean(bell_power) + eps)
        )
        tonal_score = self._compute_tonal_score(
            snr_db=snr_db,
            high_energy_ratio=high_energy_ratio,
            peak_to_median_db=peak_to_median_db,
            spectral_flatness=spectral_flatness,
        )
        is_tonal = (
            time_s >= cfg.warmup_s
            and snr_db >= cfg.min_snr_db
            and high_energy_ratio >= cfg.min_high_energy_ratio
            and peak_to_median_db >= cfg.min_peak_to_median_db
            and spectral_flatness <= cfg.max_spectral_flatness
        )
        if not is_tonal:
            self._update_noise_floor(rms_db)

        return FrameFeatures(
            time_s=float(time_s),
            rms_db=float(rms_db),
            snr_db=float(snr_db),
            high_energy_ratio=float(high_energy_ratio),
            peak_freq_hz=float(peak_freq_hz),
            peak_to_median_db=float(peak_to_median_db),
            spectral_flatness=float(spectral_flatness),
            tonal_score=float(tonal_score),
            is_tonal=bool(is_tonal),
        )

    def _compute_tonal_score(
        self,
        *,
        snr_db: float,
        high_energy_ratio: float,
        peak_to_median_db: float,
        spectral_flatness: float,
    ) -> float:
        cfg = self.config
        snr_score = np.clip((snr_db - cfg.min_snr_db) / 20.0, 0.0, 1.0)
        high_score = np.clip(
            (high_energy_ratio - cfg.min_high_energy_ratio)
            / max(1e-6, 1.0 - cfg.min_high_energy_ratio),
            0.0,
            1.0,
        )
        peak_score = np.clip(
            (peak_to_median_db - cfg.min_peak_to_median_db) / 20.0,
            0.0,
            1.0,
        )
        flatness_score = np.clip(
            (cfg.max_spectral_flatness - spectral_flatness)
            / max(1e-6, cfg.max_spectral_flatness),
            0.0,
            1.0,
        )
        return float(
            0.30 * snr_score
            + 0.25 * high_score
            + 0.30 * peak_score
            + 0.15 * flatness_score
        )

    def _update_noise_floor(self, rms_db: float) -> None:
        self._noise_history.append(float(rms_db))
        if len(self._noise_history) >= 5:
            self._noise_floor_db = float(np.percentile(self._noise_history, 20))
        else:
            self._noise_floor_db = float(np.mean(self._noise_history))

    def _update_state(self, f: FrameFeatures) -> BellEvent | None:
        cfg = self.config
        frame_duration_s = cfg.frame_size / cfg.sample_rate

        if self._state == "COOLDOWN":
            if f.time_s >= self._cooldown_until_s:
                self._state = "IDLE"
            else:
                return None

        if self._state == "IDLE":
            if f.is_tonal:
                self._start_note1(f)
            return None

        if self._state == "NOTE1":
            if f.is_tonal:
                self._extend_note(self._note1, f)
                self._missing_tonal_frames = 0
                if self._note_duration(self._note1, frame_duration_s) > cfg.max_note1_duration_s:
                    self._reset_to_idle()
                return None

            self._missing_tonal_frames += 1
            if self._missing_tonal_frames <= cfg.max_missing_tonal_frames:
                return None
            note1_duration = self._note_duration(self._note1, frame_duration_s)
            if note1_duration >= cfg.min_note1_duration_s and self._is_frequency_stable(self._note1):
                self._gap_start_s = self._note1["end_s"]
                self._state = "GAP"
            else:
                self._reset_to_idle()
            return None

        if self._state == "GAP":
            assert self._note1 is not None
            assert self._gap_start_s is not None
            gap_s = f.time_s - self._gap_start_s
            if f.is_tonal:
                if gap_s < cfg.min_gap_s:
                    self._state = "NOTE1"
                    self._extend_note(self._note1, f)
                    self._missing_tonal_frames = 0
                    return None
                if gap_s <= cfg.max_gap_s:
                    self._start_note2(f)
                    return None
                self._start_note1(f)
                return None
            if gap_s > cfg.max_gap_s:
                self._reset_to_idle()
            return None

        if self._state == "NOTE2":
            if f.is_tonal:
                self._extend_note(self._note2, f)
                self._missing_tonal_frames = 0
                if self._note_duration(self._note2, frame_duration_s) > cfg.max_note2_duration_s:
                    self._reset_to_idle()
                return None

            self._missing_tonal_frames += 1
            if self._missing_tonal_frames <= cfg.max_missing_tonal_frames:
                return None
            event = self._try_make_event(frame_duration_s)
            if event is not None:
                self._state = "COOLDOWN"
                self._cooldown_until_s = f.time_s + cfg.cooldown_s
                self._note1 = None
                self._note2 = None
                self._gap_start_s = None
                self._missing_tonal_frames = 0
                return event
            self._reset_to_idle()
            return None

        self._reset_to_idle()
        return None

    def _start_note1(self, f: FrameFeatures) -> None:
        self._note1 = self._new_note(f)
        self._note2 = None
        self._gap_start_s = None
        self._missing_tonal_frames = 0
        self._state = "NOTE1"

    def _start_note2(self, f: FrameFeatures) -> None:
        self._note2 = self._new_note(f)
        self._missing_tonal_frames = 0
        self._state = "NOTE2"

    def _new_note(self, f: FrameFeatures) -> dict:
        return {
            "start_s": f.time_s,
            "end_s": f.time_s,
            "freqs": [f.peak_freq_hz],
            "scores": [f.tonal_score],
        }

    def _extend_note(self, note: dict | None, f: FrameFeatures) -> None:
        if note is None:
            return
        note["end_s"] = f.time_s
        note["freqs"].append(f.peak_freq_hz)
        note["scores"].append(f.tonal_score)

    def _note_duration(self, note: dict | None, frame_duration_s: float) -> float:
        if note is None:
            return 0.0
        return float(note["end_s"] - note["start_s"] + frame_duration_s)

    def _note_mean_freq(self, note: dict) -> float:
        return float(np.median(note["freqs"]))

    def _note_score(self, note: dict) -> float:
        return float(np.mean(note["scores"]))

    def _is_frequency_stable(self, note: dict | None) -> bool:
        if note is None:
            return False
        freqs = np.asarray(note["freqs"], dtype=np.float32)
        if freqs.size <= 2:
            return True
        mean_freq = float(np.mean(freqs))
        std_freq = float(np.std(freqs))
        cfg = self.config
        allowed_std = max(cfg.max_freq_std_hz, cfg.max_freq_std_ratio * mean_freq)
        return std_freq <= allowed_std

    def _try_make_event(self, frame_duration_s: float) -> BellEvent | None:
        cfg = self.config
        if self._note1 is None or self._note2 is None:
            return None

        note1_duration = self._note_duration(self._note1, frame_duration_s)
        note2_duration = self._note_duration(self._note2, frame_duration_s)
        gap_s = self._note2["start_s"] - self._note1["end_s"]
        if not (cfg.min_note1_duration_s <= note1_duration <= cfg.max_note1_duration_s):
            return None
        if not (cfg.min_note2_duration_s <= note2_duration <= cfg.max_note2_duration_s):
            return None
        if not (cfg.min_gap_s <= gap_s <= cfg.max_gap_s):
            return None
        if not self._is_frequency_stable(self._note1):
            return None
        if not self._is_frequency_stable(self._note2):
            return None

        note1_freq = self._note_mean_freq(self._note1)
        note2_freq = self._note_mean_freq(self._note2)
        if not self._is_freq_relation_ok(note1_freq, note2_freq):
            return None

        event_score = float(
            0.35 * self._note_score(self._note1)
            + 0.35 * self._note_score(self._note2)
            + 0.20 * self._duration_pattern_score(note1_duration, note2_duration, gap_s)
            + 0.10 * self._frequency_relation_score(note1_freq, note2_freq)
        )
        if event_score < cfg.min_event_score:
            return None

        return BellEvent(
            start_time_s=float(self._note1["start_s"]),
            end_time_s=float(self._note2["end_s"] + frame_duration_s),
            note1_freq_hz=float(note1_freq),
            note2_freq_hz=float(note2_freq),
            note1_duration_s=float(note1_duration),
            note2_duration_s=float(note2_duration),
            gap_s=float(gap_s),
            score=event_score,
        )

    def _is_freq_relation_ok(self, note1_freq: float, note2_freq: float) -> bool:
        if self.config.freq_relation == "any":
            return True
        if self.config.freq_relation == "falling":
            return note2_freq < note1_freq * 0.98
        if self.config.freq_relation == "loose":
            return 0.50 * note1_freq <= note2_freq <= 1.30 * note1_freq
        return True

    def _duration_pattern_score(
        self,
        note1_duration: float,
        note2_duration: float,
        gap_s: float,
    ) -> float:
        cfg = self.config

        def in_range_score(x: float, low: float, high: float) -> float:
            if x < low or x > high:
                return 0.0
            center = 0.5 * (low + high)
            half = 0.5 * (high - low)
            return float(1.0 - min(abs(x - center) / max(half, 1e-6), 1.0) * 0.5)

        return float(
            (
                in_range_score(note1_duration, cfg.min_note1_duration_s, cfg.max_note1_duration_s)
                + in_range_score(note2_duration, cfg.min_note2_duration_s, cfg.max_note2_duration_s)
                + in_range_score(gap_s, cfg.min_gap_s, cfg.max_gap_s)
            )
            / 3.0
        )

    def _frequency_relation_score(self, note1_freq: float, note2_freq: float) -> float:
        if self.config.freq_relation == "falling":
            ratio = note2_freq / max(note1_freq, 1e-6)
            if 0.6 <= ratio <= 0.95:
                return 1.0
            return 0.5
        if self.config.freq_relation == "loose":
            ratio = note2_freq / max(note1_freq, 1e-6)
            if 0.7 <= ratio <= 1.1:
                return 1.0
            if 0.5 <= ratio <= 1.3:
                return 0.7
            return 0.3
        return 1.0

    def _reset_to_idle(self) -> None:
        self._state = "IDLE"
        self._note1 = None
        self._note2 = None
        self._gap_start_s = None
        self._missing_tonal_frames = 0
