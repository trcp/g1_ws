import threading
import time

from bell_sound_detector.waiter import BellWaiter


def test_wait_returns_event_notified_after_call_started():
    waiter = BellWaiter()

    def notify_later():
        time.sleep(0.02)
        waiter.notify({"score": 0.9})

    thread = threading.Thread(target=notify_later)
    thread.start()

    result = waiter.wait(timeout_s=0.5)

    thread.join(timeout=1.0)
    assert result == {"score": 0.9}


def test_wait_returns_none_after_timeout_without_event():
    waiter = BellWaiter()

    result = waiter.wait(timeout_s=0.01)

    assert result is None
