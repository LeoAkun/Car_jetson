"""Bounded replay capture handoff and all-or-close frame writes."""
from __future__ import annotations

import math
import select
import socket
import ssl
import time
from typing import Callable

from . import ProtocolError
from .latest import LatestSlot
from .source import ReplaySource
from .video_protocol import VideoFrame, encode_video_frame


class FrameWriteTimeout(TimeoutError):
    """Raised after the single total deadline for one encoded frame expires."""


class StaleSourceFrame(ProtocolError):
    """Raised when capture age cannot be trusted or exceeds its configured bound."""


def _positive_finite(value: float, field: str) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError(f"{field} must be a positive finite number")
    return float(value)


def _wait_socket(sock, *, read: bool, deadline: float, clock: Callable[[], float]) -> None:
    remaining = deadline - clock()
    if remaining <= 0:
        raise FrameWriteTimeout("video frame write deadline expired")
    readable, writable, _ = select.select(
        [sock] if read else [], [] if read else [sock], [], remaining
    )
    if not readable and not writable:
        raise FrameWriteTimeout("video frame write deadline expired")


def send_all_with_deadline(
    sock: socket.socket,
    payload: bytes,
    *,
    timeout_s: float,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Write one payload under one deadline; close on any incomplete write.

    Closing is part of the contract: once any prefix has reached the peer, a
    retry on the same stream would destroy frame boundaries.
    """

    timeout_s = _positive_finite(timeout_s, "timeout_s")
    if not isinstance(payload, bytes) or not payload:
        raise ValueError("payload must be non-empty bytes")
    deadline = clock() + timeout_s
    view = memoryview(payload)
    offset = 0
    previous_timeout = sock.gettimeout()
    sock.setblocking(False)
    try:
        while offset < len(view):
            if clock() >= deadline:
                raise FrameWriteTimeout("video frame write deadline expired")
            try:
                written = sock.send(view[offset:])
                if written == 0:
                    raise ConnectionError("video peer closed during frame write")
                offset += written
            except (BlockingIOError, ssl.SSLWantWriteError):
                _wait_socket(sock, read=False, deadline=deadline, clock=clock)
            except ssl.SSLWantReadError:
                _wait_socket(sock, read=True, deadline=deadline, clock=clock)
        sock.settimeout(previous_timeout)
    except BaseException:
        sock.close()
        raise


class ReplayFrameBuffer:
    """Bridge an independent replay producer to a network consumer.

    The handoff has depth one. A fast producer overwrites an unconsumed frame,
    so latency remains bounded instead of accumulating a FIFO backlog.
    """

    def __init__(self, source: ReplaySource):
        if not isinstance(source, ReplaySource):
            raise TypeError("source must be ReplaySource")
        self.source = source
        self.slot: LatestSlot[VideoFrame] = LatestSlot()
        self._active_generation: int | None = None

    @property
    def generation(self) -> int | None:
        return self._active_generation

    def begin_epoch(self, *, rewind: bool = False) -> int:
        generation = self.slot.clear(advance_generation=True)
        self.source.begin_epoch(rewind=rewind)
        self._active_generation = generation
        return generation

    def capture_one(self, *, generation: int) -> VideoFrame:
        if generation != self._active_generation:
            raise ProtocolError("capture worker belongs to a stale generation")
        frame = self.source.next_frame()
        if not self.slot.replace(frame, generation=generation):
            raise ProtocolError("capture worker belongs to a stale generation")
        return frame

    def take_fresh(self, *, generation: int, max_age_s: float,
                   timeout_s: float | None = None,
                   clock_ns: Callable[[], int] = time.monotonic_ns) -> VideoFrame | None:
        max_age_s = _positive_finite(max_age_s, "max_age_s")
        if generation != self._active_generation:
            raise ProtocolError("network worker belongs to a stale generation")
        item = self.slot.take_latest(timeout_s, generation=generation)
        if item is None:
            return None
        if item.generation != generation or generation != self._active_generation:
            raise ProtocolError("network worker belongs to a stale generation")
        now_ns = clock_ns()
        captured_ns = item.value.metadata["source_capture_monotonic_ns"]
        if type(now_ns) is not int or now_ns < captured_ns:
            raise StaleSourceFrame("source frame age cannot be established")
        if now_ns - captured_ns > int(max_age_s * 1_000_000_000):
            raise StaleSourceFrame("source frame exceeded age limit")
        return item.value

    def send_latest(self, sock: socket.socket, *, generation: int, max_age_s: float,
                    write_timeout_s: float, wait_timeout_s: float | None = None,
                    clock_ns: Callable[[], int] = time.monotonic_ns) -> int | None:
        frame = self.take_fresh(
            generation=generation,
            max_age_s=max_age_s,
            timeout_s=wait_timeout_s,
            clock_ns=clock_ns,
        )
        if frame is None:
            return None
        wire = encode_video_frame(frame.metadata, frame.jpeg)
        try:
            send_all_with_deadline(sock, wire, timeout_s=write_timeout_s)
        except BaseException:
            self.abort_epoch(generation=generation)
            raise
        return frame.metadata["source_frame_seq"]

    def abort_epoch(self, *, generation: int) -> int:
        if generation != self._active_generation:
            return self.slot.generation
        new_generation = self.slot.clear(advance_generation=True)
        self._active_generation = None
        return new_generation
