"""Bounded TLV1 video-frame encoding and incremental parsing."""
from __future__ import annotations

import json
import struct
from types import MappingProxyType
from dataclasses import dataclass
from typing import Any, Mapping

from . import ProtocolError


MAGIC = b"TLV1"
VERSION = 1
HEADER = struct.Struct("!4sHHII")
HEADER_SIZE = HEADER.size
DEFAULT_MAX_METADATA_BYTES = 4096
DEFAULT_MAX_JPEG_BYTES = 4 * 1024 * 1024
DEFAULT_MAX_WIDTH = 1920
DEFAULT_MAX_HEIGHT = 1080
_REQUIRED_METADATA = {
    "source_camera_id", "source_host_boot_id", "sender_instance_id", "source_boot_id",
    "source_frame_seq", "source_capture_monotonic_ns", "source_dequeue_monotonic_ns",
    "timestamp_basis", "source_mode", "width", "height", "encoding",
}
_IDENTIFIER_FIELDS = {
    "source_camera_id", "source_host_boot_id", "sender_instance_id", "source_boot_id",
    "timestamp_basis",
}
_UINT64_MAX = (1 << 64) - 1


@dataclass(frozen=True)
class VideoFrame:
    metadata: Mapping[str, Any]
    jpeg: bytes


def _bounded_text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= 128 or not value.isprintable():
        raise ProtocolError(f"invalid {field}")
    return value


def validate_metadata(metadata: Any, *, max_width: int = DEFAULT_MAX_WIDTH,
                      max_height: int = DEFAULT_MAX_HEIGHT) -> dict[str, Any]:
    if not isinstance(metadata, dict):
        raise ProtocolError("metadata must be a JSON object")
    missing = _REQUIRED_METADATA - metadata.keys()
    unknown = metadata.keys() - _REQUIRED_METADATA
    if missing:
        raise ProtocolError("missing metadata fields: " + ", ".join(sorted(missing)))
    if unknown:
        raise ProtocolError("unknown metadata fields: " + ", ".join(sorted(unknown)))
    normalized = dict(metadata)
    for field in _IDENTIFIER_FIELDS:
        normalized[field] = _bounded_text(normalized[field], field)
    if normalized["source_mode"] not in ("camera", "replay"):
        raise ProtocolError("source_mode must be camera or replay")
    if normalized["encoding"] != "jpeg":
        raise ProtocolError("encoding must be jpeg")
    for field in ("source_frame_seq", "source_capture_monotonic_ns", "source_dequeue_monotonic_ns"):
        value = normalized[field]
        if type(value) is not int or not 1 <= value <= _UINT64_MAX:
            raise ProtocolError(f"invalid {field}")
    for field, limit in (("width", max_width), ("height", max_height)):
        value = normalized[field]
        if type(value) is not int or not 1 <= value <= limit:
            raise ProtocolError(f"invalid {field}")
    return normalized


def _json_no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ProtocolError(f"invalid JSON constant: {value}")


def encode_video_frame(metadata: Mapping[str, Any], jpeg: bytes, *,
                       max_metadata_bytes: int = DEFAULT_MAX_METADATA_BYTES,
                       max_jpeg_bytes: int = DEFAULT_MAX_JPEG_BYTES,
                       max_width: int = DEFAULT_MAX_WIDTH,
                       max_height: int = DEFAULT_MAX_HEIGHT) -> bytes:
    clean = validate_metadata(dict(metadata), max_width=max_width, max_height=max_height)
    if not isinstance(jpeg, bytes) or not 1 <= len(jpeg) <= max_jpeg_bytes:
        raise ProtocolError("invalid JPEG payload length")
    try:
        encoded_metadata = json.dumps(
            clean, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        ).encode("utf-8")
    except (TypeError, ValueError, UnicodeEncodeError) as exc:
        raise ProtocolError("metadata is not strict UTF-8 JSON") from exc
    if not 1 <= len(encoded_metadata) <= max_metadata_bytes:
        raise ProtocolError("metadata length exceeds limit")
    return (HEADER.pack(MAGIC, VERSION, 0, len(encoded_metadata), len(jpeg))
            + encoded_metadata + jpeg)


class VideoFrameParser:
    """Parse TLV1 without buffering more than the current bounded frame."""

    def __init__(self, *, max_metadata_bytes: int = DEFAULT_MAX_METADATA_BYTES,
                 max_jpeg_bytes: int = DEFAULT_MAX_JPEG_BYTES,
                 max_width: int = DEFAULT_MAX_WIDTH, max_height: int = DEFAULT_MAX_HEIGHT):
        self.max_metadata_bytes = max_metadata_bytes
        self.max_jpeg_bytes = max_jpeg_bytes
        self.max_width = max_width
        self.max_height = max_height
        self._header = bytearray()
        self._metadata = bytearray()
        self._jpeg = bytearray()
        self._metadata_length = 0
        self._jpeg_length = 0
        self._decoded_metadata: dict[str, Any] | None = None
        self._state = "header"
        self._failed = False
        self._closed = False

    def _fail(self, message: str):
        self._failed = True
        self._header.clear()
        self._metadata.clear()
        self._jpeg.clear()
        self._decoded_metadata = None
        raise ProtocolError(message)

    @staticmethod
    def _copy_field(target: bytearray, source: memoryview, offset: int, size: int) -> int:
        amount = min(size - len(target), len(source) - offset)
        if amount > 0:
            target.extend(source[offset:offset + amount])
            offset += amount
        return offset

    def feed(self, data: bytes | bytearray | memoryview) -> list[VideoFrame]:
        if self._failed or self._closed:
            raise ProtocolError("parser is closed")
        if not isinstance(data, (bytes, bytearray, memoryview)):
            self._fail("input must be bytes")
        source = memoryview(data).cast("B")
        offset = 0
        frames: list[VideoFrame] = []
        try:
            while offset < len(source):
                if self._state == "header":
                    offset = self._copy_field(self._header, source, offset, HEADER_SIZE)
                    if len(self._header) < HEADER_SIZE:
                        break
                    magic, version, flags, metadata_length, jpeg_length = HEADER.unpack(self._header)
                    if magic != MAGIC or version != VERSION or flags != 0:
                        self._fail("invalid TLV1 header")
                    if not 1 <= metadata_length <= self.max_metadata_bytes:
                        self._fail("metadata length exceeds limit")
                    if not 1 <= jpeg_length <= self.max_jpeg_bytes:
                        self._fail("JPEG length exceeds limit")
                    self._metadata_length = metadata_length
                    self._jpeg_length = jpeg_length
                    self._state = "metadata"
                elif self._state == "metadata":
                    offset = self._copy_field(self._metadata, source, offset, self._metadata_length)
                    if len(self._metadata) < self._metadata_length:
                        break
                    try:
                        metadata = json.loads(
                            self._metadata.decode("utf-8"),
                            object_pairs_hook=_json_no_duplicates,
                            parse_constant=_reject_constant,
                        )
                        metadata = validate_metadata(metadata, max_width=self.max_width,
                                                      max_height=self.max_height)
                    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                        self._fail(f"invalid metadata JSON: {exc}")
                    except ProtocolError as exc:
                        self._fail(str(exc))
                    self._decoded_metadata = metadata
                    self._state = "jpeg"
                else:
                    offset = self._copy_field(self._jpeg, source, offset, self._jpeg_length)
                    if len(self._jpeg) < self._jpeg_length:
                        break
                    if self._decoded_metadata is None:  # defensive state invariant
                        self._fail("metadata state was lost")
                    frames.append(VideoFrame(MappingProxyType(self._decoded_metadata), bytes(self._jpeg)))
                    self._header.clear()
                    self._metadata.clear()
                    self._jpeg.clear()
                    self._decoded_metadata = None
                    self._metadata_length = self._jpeg_length = 0
                    self._state = "header"
            return frames
        except ProtocolError:
            raise
        except Exception as exc:
            self._fail(f"video parser failure: {type(exc).__name__}: {exc}")

    def finish(self) -> None:
        """Mark EOF; incomplete frames are protocol errors and cannot be resumed."""
        if self._failed or self._closed:
            raise ProtocolError("parser is closed")
        if self._state != "header" or self._header:
            self._fail("EOF in incomplete video frame")
        self._closed = True

