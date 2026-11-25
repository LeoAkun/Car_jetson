"""Bounded reconnect delay state shared by staging endpoints."""
from __future__ import annotations

import math
import random
import time
from typing import Callable


class ExponentialBackoff:
    """Generate bounded exponential delays with optional symmetric jitter."""

    def __init__(self, *, initial_s: float = 0.1, maximum_s: float = 5.0,
                 multiplier: float = 2.0, jitter_fraction: float = 0.2,
                 random_value: Callable[[], float] = random.random):
        values = (initial_s, maximum_s, multiplier, jitter_fraction)
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            raise ValueError("backoff values must be finite numbers")
        if initial_s <= 0 or maximum_s < initial_s or multiplier <= 1:
            raise ValueError("invalid backoff bounds or multiplier")
        if not 0 <= jitter_fraction < 1:
            raise ValueError("jitter_fraction must be in [0, 1)")
        if not callable(random_value):
            raise TypeError("random_value must be callable")
        self.initial_s = float(initial_s)
        self.maximum_s = float(maximum_s)
        self.multiplier = float(multiplier)
        self.jitter_fraction = float(jitter_fraction)
        self.random_value = random_value
        self._attempt = 0
        self._base = self.initial_s

    @property
    def attempt(self) -> int:
        return self._attempt

    def next_delay(self) -> float:
        raw_random = self.random_value()
        if (isinstance(raw_random, bool) or not isinstance(raw_random, (int, float))
                or not math.isfinite(raw_random) or not 0 <= raw_random <= 1):
            raise ValueError("random_value must return a finite value in [0, 1]")
        base = self._base
        self._attempt += 1
        self._base = min(self.maximum_s, base * self.multiplier)
        jitter = (2 * float(raw_random) - 1) * self.jitter_fraction
        return base * (1 + jitter)

    def reset(self) -> None:
        self._attempt = 0
        self._base = self.initial_s


def connect_with_backoff(connect: Callable[[], object], *, max_attempts: int,
                         backoff: ExponentialBackoff,
                         sleep: Callable[[float], None] = time.sleep,
                         deadline: float | None = None,
                         clock: Callable[[], float] = time.monotonic):
    """Retry only the caller's TCP-connect operation under a finite attempt cap."""

    if type(max_attempts) is not int or max_attempts < 1:
        raise ValueError("max_attempts must be a positive integer")
    if not isinstance(backoff, ExponentialBackoff):
        raise TypeError("backoff must be ExponentialBackoff")
    if not callable(connect) or not callable(sleep) or not callable(clock):
        raise TypeError("connect, sleep, and clock must be callable")
    if (deadline is not None
            and (isinstance(deadline, bool) or not isinstance(deadline, (int, float))
                 or not math.isfinite(deadline))):
        raise ValueError("deadline must be a finite monotonic value or None")
    for attempt in range(1, max_attempts + 1):
        if deadline is not None and clock() >= deadline:
            raise TimeoutError("TCP connect retry deadline expired")
        try:
            result = connect()
            backoff.reset()
            return result
        except (ConnectionError, TimeoutError, OSError):
            if attempt == max_attempts:
                raise
            delay = backoff.next_delay()
            if deadline is not None and clock() + delay >= deadline:
                raise TimeoutError("TCP connect retry deadline expired")
            sleep(delay)
    raise AssertionError("unreachable")
