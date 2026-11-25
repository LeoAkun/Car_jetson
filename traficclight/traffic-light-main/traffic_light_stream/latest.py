"""A depth-one, generation-aware handoff slot."""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Generic, Optional, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class SlotItem(Generic[T]):
    generation: int
    value: T


class LatestSlot(Generic[T]):
    """Keep only the newest pending value and reject stale generations.

    Values should be immutable snapshots. A consumed item is no longer owned by
    the slot, so producers may safely publish the next item without mutating it.
    """

    def __init__(self):
        self._condition = threading.Condition()
        self._generation = 0
        self._item: Optional[SlotItem[T]] = None
        self._closed = False
        self._dropped_count = 0

    @property
    def generation(self) -> int:
        with self._condition:
            return self._generation

    @property
    def dropped_count(self) -> int:
        with self._condition:
            return self._dropped_count

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._closed

    def replace(self, value: T, *, generation: int) -> bool:
        """Publish a value; return False when it belongs to an old generation."""
        with self._condition:
            if self._closed:
                return False
            if type(generation) is not int or generation != self._generation:
                return False
            if self._item is not None:
                self._dropped_count += 1
            self._item = SlotItem(generation, value)
            self._condition.notify()
            return True

    def take_latest(self, timeout: float | None = None, *,
                    generation: int | None = None) -> SlotItem[T] | None:
        """Wait for the newest value, optionally only for one generation.

        A generation-bound waiter returns ``None`` as soon as reconnect or
        invalidation advances the slot. This prevents an old worker from
        sleeping through a disconnect and consuming data from its successor.
        """
        if timeout is not None:
            try:
                valid_timeout = (not isinstance(timeout, bool)
                                 and isinstance(timeout, (int, float))
                                 and math.isfinite(timeout) and timeout >= 0)
            except OverflowError:
                valid_timeout = False
            if not valid_timeout:
                raise ValueError("timeout must be a finite non-negative number or None")
        if generation is not None and type(generation) is not int:
            raise ValueError("generation must be an integer or None")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while (self._item is None and not self._closed
                   and (generation is None or generation == self._generation)):
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return None
                self._condition.wait(remaining)
            if generation is not None and generation != self._generation:
                return None
            if self._closed or self._item is None:
                return None
            item, self._item = self._item, None
            if item.generation != self._generation:
                return None
            return item

    def clear(self, *, advance_generation: bool = False) -> int:
        """Clear pending data and optionally invalidate all in-flight work."""
        if type(advance_generation) is not bool:
            raise TypeError("advance_generation must be bool")
        with self._condition:
            self._item = None
            if advance_generation:
                self._generation += 1
            self._condition.notify_all()
            return self._generation

    def close(self) -> None:
        """Wake waiters and reject future publications."""
        with self._condition:
            self._closed = True
            self._item = None
            self._condition.notify_all()

