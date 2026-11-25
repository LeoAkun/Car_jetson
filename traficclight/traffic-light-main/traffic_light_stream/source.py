"""Deterministic replay source used before the physical camera is admitted."""
from __future__ import annotations

from pathlib import Path
import time
import uuid
from types import MappingProxyType
from typing import Callable, Iterable

from . import ProtocolError
from .video_protocol import VideoFrame, validate_metadata


REPLAY_CAMERA_ID_PREFIX = "replay:"
REPLAY_TIMESTAMP_BASIS = "replay_monotonic_generated"


def read_host_boot_id(path: str = "/proc/sys/kernel/random/boot_id") -> str:
    """Read the Linux boot identity used to disambiguate monotonic clocks."""

    try:
        value = Path(path).read_text(encoding="ascii").strip()
        return str(uuid.UUID(value))
    except (OSError, UnicodeError, ValueError) as exc:
        raise ProtocolError("cannot read a valid host boot id") from exc


class ReplaySource:
    """Yield immutable TLV1 frames from a finite set of JPEG fixtures.

    A replay identity must begin with ``replay:``.  Each ``begin_epoch`` call
    creates a new source boot id and resets its sequence counter.  Reconnection
    therefore cannot make buffered data from an earlier transport epoch look
    current.
    """

    def __init__(
        self,
        jpeg_frames: Iterable[bytes],
        *,
        camera_id: str,
        width: int,
        height: int,
        host_boot_id: str | None = None,
        sender_instance_id: str | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ):
        frames = tuple(jpeg_frames)
        if not frames or any(not isinstance(frame, bytes) or not frame for frame in frames):
            raise ProtocolError("replay source requires non-empty byte frames")
        if not isinstance(camera_id, str) or not camera_id.startswith(REPLAY_CAMERA_ID_PREFIX):
            raise ProtocolError("replay camera_id must begin with replay:")
        self._frames = frames
        self._camera_id = camera_id
        self._width = width
        self._height = height
        self._host_boot_id = host_boot_id or read_host_boot_id()
        self._sender_instance_id = sender_instance_id or str(uuid.uuid4())
        self._clock_ns = clock_ns
        self._source_boot_id: str | None = None
        self._sequence = 0
        self._index = 0
        self._last_capture_ns = 0

        # Reuse the frozen wire contract for all static constructor fields.
        validate_metadata(self._metadata(1, 1, str(uuid.uuid4())))

    @property
    def camera_id(self) -> str:
        return self._camera_id

    @property
    def source_boot_id(self) -> str | None:
        return self._source_boot_id

    @property
    def sender_instance_id(self) -> str:
        return self._sender_instance_id

    @property
    def remaining(self) -> int:
        return len(self._frames) - self._index

    def _metadata(self, sequence: int, capture_ns: int, source_boot_id: str) -> dict:
        return {
            "source_camera_id": self._camera_id,
            "source_host_boot_id": self._host_boot_id,
            "sender_instance_id": self._sender_instance_id,
            "source_boot_id": source_boot_id,
            "source_frame_seq": sequence,
            "source_capture_monotonic_ns": capture_ns,
            "source_dequeue_monotonic_ns": capture_ns,
            "timestamp_basis": REPLAY_TIMESTAMP_BASIS,
            "source_mode": "replay",
            "width": self._width,
            "height": self._height,
            "encoding": "jpeg",
        }

    def begin_epoch(self, *, rewind: bool = False) -> str:
        if type(rewind) is not bool:
            raise TypeError("rewind must be bool")
        if rewind:
            self._index = 0
        self._source_boot_id = str(uuid.uuid4())
        self._sequence = 0
        self._last_capture_ns = 0
        return self._source_boot_id

    def next_frame(self) -> VideoFrame:
        if self._source_boot_id is None:
            raise ProtocolError("replay epoch has not started")
        if self._index >= len(self._frames):
            raise StopIteration
        capture_ns = self._clock_ns()
        if type(capture_ns) is not int or capture_ns < 1:
            raise ProtocolError("monotonic clock returned an invalid value")
        # A coarse or injected clock may repeat. Preserve the wire invariant.
        capture_ns = max(capture_ns, self._last_capture_ns + 1)
        self._last_capture_ns = capture_ns
        self._sequence += 1
        metadata = validate_metadata(
            self._metadata(self._sequence, capture_ns, self._source_boot_id)
        )
        jpeg = self._frames[self._index]
        self._index += 1
        return VideoFrame(MappingProxyType(metadata), jpeg)
