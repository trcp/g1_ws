import numpy as np

from bell_sound_detector.bell_detector import BellDetector


def _tone(freq_hz, duration_s, sample_rate=16000, amplitude=0.6):
    samples = int(duration_s * sample_rate)
    t = np.arange(samples, dtype=np.float32) / sample_rate
    envelope = np.hanning(samples).astype(np.float32)
    return amplitude * np.sin(2.0 * np.pi * freq_hz * t) * envelope


def test_silence_does_not_detect_bell():
    detector = BellDetector()
    audio = np.zeros(16000, dtype=np.int16)

    events = detector.predict(audio)

    assert events == []


def test_two_tone_bell_is_detected_from_int16_audio():
    detector = BellDetector()
    sample_rate = detector.config.sample_rate
    warmup = np.zeros(int(0.4 * sample_rate), dtype=np.float32)
    note1 = _tone(2200.0, 0.18, sample_rate)
    gap = np.zeros(int(0.16 * sample_rate), dtype=np.float32)
    note2 = _tone(1500.0, 0.35, sample_rate)
    tail = np.zeros(int(0.4 * sample_rate), dtype=np.float32)
    audio = np.concatenate([warmup, note1, gap, note2, tail])
    pcm = np.clip(audio * 32767.0, -32768, 32767).astype(np.int16)

    events = []
    for start in range(0, pcm.size, 320):
        events.extend(detector.predict(pcm[start : start + 320]))

    assert len(events) == 1
    event = events[0]
    assert 2000.0 <= event.note1_freq_hz <= 2400.0
    assert 1300.0 <= event.note2_freq_hz <= 1700.0
    assert event.score >= detector.config.min_event_score
