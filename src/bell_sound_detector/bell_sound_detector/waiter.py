from __future__ import annotations

import time
from threading import Condition
from typing import Any


class BellWaiter:
    def __init__(self) -> None:
        self._condition = Condition()
        self._generation = 0
        self._last_event: Any = None

    def notify(self, event: Any) -> None:
        with self._condition:
            self._generation += 1
            self._last_event = event
            self._condition.notify_all()

    def wait(self, timeout_s: float) -> Any | None:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._condition:
            start_generation = self._generation
            while self._generation == start_generation:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)
            return self._last_event
