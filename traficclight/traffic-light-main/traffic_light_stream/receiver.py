"""Connection-generation-aware TLV1 video receiver state machine."""
from __future__ import annotations

from dataclasses import dataclass
import time
from types import MappingProxyType
from typing import Any, Callable, Mapping

from . import ProtocolError
from .latest import LatestSlot, SlotItem
from .source import REPLAY_CAMERA_ID_PREFIX
from .video_protocol import VideoFrameParser


_IMMUTABLE_CONNECTION_FIELDS = (
    "source_camera_id", "source_host_boot_id", "sender_instance_id",
    "source_boot_id", "timestamp_basis", "source_mode", "width", "height",
    "encoding",
)


@dataclass(frozen=True)
class ReceivedVideoFrame:
    metadata: Mapping[str, Any]
    jpeg: bytes
    peer_role: str
    connection_generation: int
    receive_monotonic_ns: int


def _has_jpeg_markers(payload: bytes) -> bool:
    return len(payload) >= 4 and payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9")


class VideoReceiver:
    """Validate one authenticated sender and publish only its newest frame."""

    def __init__(
        self,
        *,
        expected_camera_id: str,
        expected_source_mode: str,
        jpeg_validator: Callable[[bytes], bool] = _has_jpeg_markers,
        max_metadata_bytes: int = 4096,
        max_jpeg_bytes: int = 4 * 1024 * 1024,
        max_width: int = 1920,
        max_height: int = 1080,
    ):
        if expected_source_mode not in ("camera", "replay"):
            raise ProtocolError("expected_source_mode must be camera or replay")
        is_replay_identity = (isinstance(expected_camera_id, str)
                              and expected_camera_id.startswith(REPLAY_CAMERA_ID_PREFIX))
        if expected_source_mode == "replay" and not is_replay_identity:
            raise ProtocolError("replay receiver requires a replay: camera identity")
        if expected_source_mode == "camera" and is_replay_identity:
            raise ProtocolError("camera receiver cannot use a replay identity")
        if not callable(jpeg_validator):
            raise TypeError("jpeg_validator must be callable")
        self.expected_camera_id = expected_camera_id
        self.expected_source_mode = expected_source_mode
        self.jpeg_validator = jpeg_validator
        self.output: LatestSlot[ReceivedVideoFrame] = LatestSlot()
        self._parser_limits = dict(
            max_metadata_bytes=max_metadata_bytes,
            max_jpeg_bytes=max_jpeg_bytes,
            max_width=max_width,
            max_height=max_height,
        )
        self._active_generation: int | None = None
        self._parser: VideoFrameParser | None = None
        self._identity: tuple[Any, ...] | None = None
        self._last_sequence = 0
        self._last_capture_ns = 0

    @property
    def active_generation(self) -> int | None:
        return self._active_generation

    def begin_connection(self, *, peer_role: str) -> int:
        if peer_role != "video-sender":
            raise ProtocolError("TLS peer is not authorized as video-sender")
        if self._active_generation is not None:
            raise ProtocolError("a video source connection is already active")
        generation = self.output.clear(advance_generation=True)
        self._active_generation = generation
        self._parser = VideoFrameParser(**self._parser_limits)
        self._identity = None
        self._last_sequence = 0
        self._last_capture_ns = 0
        return generation

    def _invalidate(self, generation: int) -> int:
        if generation != self._active_generation:
            return self.output.generation
        self._active_generation = None
        self._parser = None
        self._identity = None
        self._last_sequence = 0
        self._last_capture_ns = 0
        return self.output.clear(advance_generation=True)

    def _validate_frame(self, frame, *, generation: int,
                        receive_monotonic_ns: int) -> ReceivedVideoFrame:
        metadata = frame.metadata
        if metadata["source_camera_id"] != self.expected_camera_id:
            raise ProtocolError("source camera identity changed or is not admitted")
        if metadata["source_mode"] != self.expected_source_mode:
            raise ProtocolError("source mode changed or is not admitted")
        if metadata["source_dequeue_monotonic_ns"] < metadata["source_capture_monotonic_ns"]:
            raise ProtocolError("source dequeue time precedes capture time")
        if not self.jpeg_validator(frame.jpeg):
            raise ProtocolError("invalid JPEG payload")

        identity = tuple(metadata[field] for field in _IMMUTABLE_CONNECTION_FIELDS)
        if self._identity is None:
            self._identity = identity
        elif identity != self._identity:
            raise ProtocolError("source identity or frame shape changed within a connection")
        sequence = metadata["source_frame_seq"]
        capture_ns = metadata["source_capture_monotonic_ns"]
        if sequence <= self._last_sequence:
            raise ProtocolError("source frame sequence is not strictly increasing")
        if capture_ns <= self._last_capture_ns:
            raise ProtocolError("source capture time is not strictly increasing")
        self._last_sequence = sequence
        self._last_capture_ns = capture_ns
        return ReceivedVideoFrame(
            MappingProxyType(dict(metadata)), frame.jpeg, "video-sender", generation,
            receive_monotonic_ns,
        )

    def feed(self, data: bytes, *, generation: int,
             receive_monotonic_ns: int | None = None) -> tuple[ReceivedVideoFrame, ...]:
        if generation != self._active_generation or self._parser is None:
            raise ProtocolError("video data belongs to an inactive connection generation")
        receive_ns = time.monotonic_ns() if receive_monotonic_ns is None else receive_monotonic_ns
        if type(receive_ns) is not int or receive_ns < 1:
            self._invalidate(generation)
            raise ProtocolError("invalid receive monotonic time")
        try:
            accepted = []
            for frame in self._parser.feed(data):
                verified = self._validate_frame(
                    frame, generation=generation, receive_monotonic_ns=receive_ns
                )
                if not self.output.replace(verified, generation=generation):
                    raise ProtocolError("receiver generation changed during publication")
                accepted.append(verified)
            return tuple(accepted)
        except BaseException:
            self._invalidate(generation)
            raise

    def end_connection(self, *, generation: int) -> int:
        """Process EOF and invalidate all pending/in-flight work from the peer."""

        if generation != self._active_generation or self._parser is None:
            raise ProtocolError("video connection generation is not active")
        try:
            self._parser.finish()
        finally:
            new_generation = self._invalidate(generation)
        return new_generation

    def abort_connection(self, *, generation: int) -> int:
        """Handle RST/read failure without attempting to resume a partial frame."""

        if generation != self._active_generation:
            raise ProtocolError("video connection generation is not active")
        return self._invalidate(generation)

    def take_latest(self, timeout: float | None = None) -> SlotItem[ReceivedVideoFrame] | None:
        return self.output.take_latest(timeout)
