"""Strict schema-2 NDJSON validation and incremental line framing."""
from __future__ import annotations

import json
import math
import re
import time
from typing import Any, Callable

from . import ProtocolError


SCHEMA_VERSION = 2
DEFAULT_MAX_LINE_BYTES = 65536
_UINT64_MAX = (1 << 64) - 1
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_COMMON = {"schema_version", "type"}
_MESSAGE_FIELDS = {
    "hello": _COMMON | {"role", "peer_id", "peer_boot_id", "capabilities"},
    "status": _COMMON | {"request_id"},
    "command": _COMMON | {"action", "request_id"},
    "ack": _COMMON | {"action", "request_id", "accepted", "reason", "service_boot_id"},
    "observation": _COMMON | {
        "service_boot_id", "seq", "session_id", "task_id", "map_name", "waypoint_id",
        "direction", "model_sha256", "roi_norm", "source_camera_id", "source_host_boot_id",
        "source_boot_id", "source_frame_seq", "source_capture_monotonic_ns",
        "timestamp_basis", "source_mode", "receive_monotonic_ns", "state", "valid",
        "confidence", "has_bbox", "bbox_norm", "reason", "inference_ms", "control_eligible",
        "automatic_release_enabled", "thresholds_calibrated",
    },
    "heartbeat": _COMMON | {
        "service_boot_id", "heartbeat_seq", "ready", "armed", "session_id", "camera_ok",
        "model_ok", "model_validation_status", "control_eligible", "automatic_release_enabled",
        "thresholds_calibrated", "source_camera_id", "source_host_boot_id", "source_boot_id",
        "receive_fps", "inference_fps", "send_fps", "source_queue_drops", "result_queue_drops",
        "queue_depth", "last_frame_age_ms", "reason",
    },
    "error": _COMMON | {"code", "message"},
}
_OPTIONAL_FIELDS = {
    "ack": {"session_id", "lease_duration_ms", "bindings"},
    "error": {"request_id"},
}
_ACTION_FIELDS = {
    "arm": {
        "session_id", "task_id", "map_name", "waypoint_id", "direction", "roi_norm",
        "expected_model_sha256", "expected_detector_boot_id", "source_camera_id",
        "source_host_boot_id", "source_boot_id", "arm_monotonic_ns",
    },
    "renew": {"session_id"},
    "disarm": {"session_id"},
}


def _text(value: Any, field: str, maximum: int = 128) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= maximum or not value.isprintable():
        raise ProtocolError(f"invalid {field}")
    return value


def _uint(value: Any, field: str, maximum: int = _UINT64_MAX) -> int:
    if type(value) is not int or not 0 <= value <= maximum:
        raise ProtocolError(f"invalid {field}")
    return value


def _positive_uint(value: Any, field: str, maximum: int = _UINT64_MAX) -> int:
    parsed = _uint(value, field, maximum)
    if parsed == 0:
        raise ProtocolError(f"invalid {field}")
    return parsed


def _boolean(value: Any, field: str) -> bool:
    if type(value) is not bool:
        raise ProtocolError(f"invalid {field}")
    return value


def _number(value: Any, field: str, *, minimum: float = 0.0,
            maximum: float = float("inf")) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"invalid {field}")
    try:
        valid_number = math.isfinite(value) and minimum <= value <= maximum
    except OverflowError:
        valid_number = False
    if not valid_number:
        raise ProtocolError(f"invalid {field}")
    return float(value)


def _nullable_number(value: Any, field: str) -> None:
    if value is not None:
        _number(value, field)


def _roi(value: Any, field: str) -> None:
    if not isinstance(value, list) or len(value) != 4:
        raise ProtocolError(f"invalid {field}")
    coords = [_number(component, field, minimum=0, maximum=1) for component in value]
    if not (coords[0] < coords[2] and coords[1] < coords[3]):
        raise ProtocolError(f"invalid {field}")


def _false_control_flags(message: dict[str, Any]) -> None:
    for name in ("control_eligible", "automatic_release_enabled", "thresholds_calibrated"):
        if _boolean(message[name], name):
            raise ProtocolError(f"{name} must remain false for the candidate model")


def _strict_json_value(value: Any, *, depth: int = 0) -> None:
    if depth > 4:
        raise ProtocolError("JSON value nesting exceeds limit")
    if value is None or type(value) in (bool, str):
        if isinstance(value, str) and len(value) > 256:
            raise ProtocolError("JSON string exceeds limit")
        return
    if type(value) is int:
        if not -(1 << 63) <= value <= _UINT64_MAX:
            raise ProtocolError("JSON integer exceeds limit")
        return
    if type(value) is float:
        if not math.isfinite(value):
            raise ProtocolError("non-finite JSON number")
        return
    if isinstance(value, list):
        if len(value) > 64:
            raise ProtocolError("JSON array exceeds limit")
        for item in value:
            _strict_json_value(item, depth=depth + 1)
        return
    if isinstance(value, dict):
        if len(value) > 32:
            raise ProtocolError("JSON object exceeds limit")
        for key, item in value.items():
            if not isinstance(key, str) or len(key) > 128:
                raise ProtocolError("invalid JSON object key")
            _strict_json_value(item, depth=depth + 1)
        return
    raise ProtocolError("unsupported JSON value")


def _validate_message(message: Any) -> dict[str, Any]:
    if not isinstance(message, dict):
        raise ProtocolError("message must be a JSON object")
    if type(message.get("schema_version")) is not int or message["schema_version"] != SCHEMA_VERSION:
        raise ProtocolError("unsupported schema_version")
    kind = message.get("type")
    if not isinstance(kind, str) or kind not in _MESSAGE_FIELDS:
        raise ProtocolError("unknown message type")
    required = set(_MESSAGE_FIELDS[kind])
    optional = _OPTIONAL_FIELDS.get(kind, set())
    allowed = required | optional
    if kind == "command":
        allowed |= set().union(*_ACTION_FIELDS.values())
    missing, unknown = required - message.keys(), message.keys() - allowed
    if missing:
        raise ProtocolError("missing fields: " + ", ".join(sorted(missing)))
    if unknown:
        raise ProtocolError("unknown fields: " + ", ".join(sorted(unknown)))

    if kind == "hello":
        if message["role"] not in ("sender", "controller", "service"):
            raise ProtocolError("invalid role")
        _text(message["peer_id"], "peer_id")
        _text(message["peer_boot_id"], "peer_boot_id")
        capabilities = message["capabilities"]
        if (not isinstance(capabilities, list) or len(capabilities) > 32
                or any(not isinstance(item, str) or not 1 <= len(item) <= 64
                       or not item.isprintable() for item in capabilities)
                or len(set(capabilities)) != len(capabilities)):
            raise ProtocolError("invalid capabilities")
    elif kind == "status":
        _text(message["request_id"], "request_id")
    elif kind == "command":
        _text(message["request_id"], "request_id")
        action = message["action"]
        if not isinstance(action, str) or action not in _ACTION_FIELDS:
            raise ProtocolError("unknown command action")
        expected = required | _ACTION_FIELDS[action]
        missing, unknown = expected - message.keys(), message.keys() - expected
        if missing:
            raise ProtocolError("missing command fields: " + ", ".join(sorted(missing)))
        if unknown:
            raise ProtocolError("unknown command fields: " + ", ".join(sorted(unknown)))
        _text(message["session_id"], "session_id")
        if action == "arm":
            for field in ("task_id", "map_name", "direction", "expected_detector_boot_id",
                          "source_camera_id", "source_host_boot_id", "source_boot_id"):
                _text(message[field], field)
            _uint(message["waypoint_id"], "waypoint_id", (1 << 32) - 1)
            _roi(message["roi_norm"], "roi_norm")
            if not isinstance(message["expected_model_sha256"], str) or not _SHA256.fullmatch(
                    message["expected_model_sha256"]):
                raise ProtocolError("invalid expected_model_sha256")
            _positive_uint(message["arm_monotonic_ns"], "arm_monotonic_ns", _UINT64_MAX)
    elif kind == "ack":
        _text(message["request_id"], "request_id")
        if message["action"] not in ("status", "arm", "renew", "disarm"):
            raise ProtocolError("invalid ack action")
        accepted = _boolean(message["accepted"], "accepted")
        if not isinstance(message["reason"], str) or len(message["reason"]) > 256:
            raise ProtocolError("invalid reason")
        if not accepted and not message["reason"]:
            raise ProtocolError("rejected ack requires a reason")
        _text(message["service_boot_id"], "service_boot_id")
        if "session_id" in message:
            _text(message["session_id"], "session_id")
        if "lease_duration_ms" in message:
            _positive_uint(message["lease_duration_ms"], "lease_duration_ms", 120000)
        if "bindings" in message and (not isinstance(message["bindings"], dict)
                                       or len(message["bindings"]) > 16):
            raise ProtocolError("invalid bindings")
        if "bindings" in message:
            _strict_json_value(message["bindings"])
    elif kind == "observation":
        for field in ("service_boot_id", "session_id", "task_id", "map_name", "direction",
                      "source_camera_id", "source_host_boot_id", "source_boot_id",
                      "timestamp_basis", "reason"):
            _text(message[field], field, 256 if field == "reason" else 128)
        _positive_uint(message["seq"], "seq", _UINT64_MAX)
        _uint(message["waypoint_id"], "waypoint_id", (1 << 32) - 1)
        _positive_uint(message["source_frame_seq"], "source_frame_seq")
        _positive_uint(message["source_capture_monotonic_ns"], "source_capture_monotonic_ns")
        _positive_uint(message["receive_monotonic_ns"], "receive_monotonic_ns")
        if message["source_mode"] not in ("camera", "replay"):
            raise ProtocolError("invalid source_mode")
        if not isinstance(message["model_sha256"], str) or not _SHA256.fullmatch(message["model_sha256"]):
            raise ProtocolError("invalid model_sha256")
        _roi(message["roi_norm"], "roi_norm")
        if message["state"] not in ("RED", "GREEN", "UNKNOWN"):
            raise ProtocolError("invalid state")
        valid = _boolean(message["valid"], "valid")
        has_bbox = _boolean(message["has_bbox"], "has_bbox")
        if valid != (message["state"] != "UNKNOWN") or has_bbox != valid:
            raise ProtocolError("observation state, valid, and has_bbox disagree")
        _number(message["confidence"], "confidence", maximum=1)
        _number(message["inference_ms"], "inference_ms", maximum=60000)
        bbox = message["bbox_norm"]
        if not isinstance(bbox, list) or len(bbox) != 4:
            raise ProtocolError("invalid bbox_norm")
        coords = [_number(component, "bbox_norm", maximum=1) for component in bbox]
        if has_bbox and not (coords[0] < coords[2] and coords[1] < coords[3]):
            raise ProtocolError("invalid bbox_norm geometry")
        if not has_bbox and any(coords):
            raise ProtocolError("bbox_norm must be zero when has_bbox is false")
        _false_control_flags(message)
    elif kind == "heartbeat":
        _text(message["service_boot_id"], "service_boot_id")
        _positive_uint(message["heartbeat_seq"], "heartbeat_seq")
        ready = _boolean(message["ready"], "ready")
        armed = _boolean(message["armed"], "armed")
        camera_ok = _boolean(message["camera_ok"], "camera_ok")
        model_ok = _boolean(message["model_ok"], "model_ok")
        for field in ("session_id", "source_camera_id", "source_host_boot_id", "source_boot_id"):
            if message[field] is not None:
                _text(message[field], field)
        if message["model_validation_status"] not in ("candidate", "validated", "unavailable"):
            raise ProtocolError("invalid model_validation_status")
        for field in ("receive_fps", "inference_fps", "send_fps", "last_frame_age_ms"):
            _nullable_number(message[field], field)
        for field in ("source_queue_drops", "result_queue_drops", "queue_depth"):
            _uint(message[field], field, (1 << 32) - 1)
        if ready and not (camera_ok and model_ok):
            raise ProtocolError("ready requires camera_ok and model_ok")
        if armed and (not ready or message["session_id"] is None):
            raise ProtocolError("armed requires ready and session_id")
        _text(message["reason"], "reason", 256)
        _false_control_flags(message)
    else:
        _text(message["code"], "code", 64)
        _text(message["message"], "message", 256)
        if "request_id" in message:
            _text(message["request_id"], "request_id")
    return dict(message)


def validate_message(message: Any) -> dict[str, Any]:
    return _validate_message(message)


def encode_message(message: Any, *, max_line_bytes: int = DEFAULT_MAX_LINE_BYTES) -> bytes:
    clean = _validate_message(message)
    try:
        line = json.dumps(clean, ensure_ascii=False, allow_nan=False,
                          separators=(",", ":")).encode("utf-8") + b"\n"
    except (TypeError, ValueError, UnicodeEncodeError) as exc:
        raise ProtocolError("message is not strict UTF-8 JSON") from exc
    if len(line) > max_line_bytes:
        raise ProtocolError("NDJSON line exceeds limit")
    return line


def _object_no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ProtocolError(f"invalid JSON constant: {value}")


class NDJSONParser:
    """Bounded byte parser with one total deadline per unfinished line."""

    def __init__(self, *, max_line_bytes: int = DEFAULT_MAX_LINE_BYTES,
                 line_timeout_s: float = 0.5, clock: Callable[[], float] = time.monotonic):
        if type(max_line_bytes) is not int or not 2 <= max_line_bytes <= DEFAULT_MAX_LINE_BYTES:
            raise ValueError("invalid max_line_bytes")
        if isinstance(line_timeout_s, bool) or not isinstance(line_timeout_s, (int, float)):
            raise ValueError("invalid line_timeout_s")
        try:
            valid_timeout = math.isfinite(line_timeout_s) and line_timeout_s > 0
        except OverflowError:
            valid_timeout = False
        if not valid_timeout:
            raise ValueError("invalid line_timeout_s")
        self.max_line_bytes = max_line_bytes
        self.line_timeout_s = float(line_timeout_s)
        self.clock = clock
        self._buffer = bytearray()
        self._deadline: float | None = None
        self._closed = False

    def _fail(self, message: str):
        self._closed = True
        self._buffer.clear()
        self._deadline = None
        raise ProtocolError(message)

    def _parse_line(self, line: bytes) -> dict[str, Any]:
        if not line:
            self._fail("empty NDJSON line")
        try:
            message = json.loads(line.decode("utf-8"), object_pairs_hook=_object_no_duplicates,
                                 parse_constant=_reject_constant)
        except (UnicodeDecodeError, json.JSONDecodeError, RecursionError, ValueError) as exc:
            self._fail(f"invalid NDJSON JSON: {exc}")
        except ProtocolError as exc:
            self._fail(str(exc))
        try:
            return _validate_message(message)
        except ProtocolError as exc:
            self._fail(str(exc))

    def feed(self, data: bytes | bytearray | memoryview) -> list[dict[str, Any]]:
        if self._closed:
            raise ProtocolError("parser is closed")
        if not isinstance(data, (bytes, bytearray, memoryview)):
            self._fail("input must be bytes")
        now = self.clock()
        if self._deadline is not None and now >= self._deadline:
            self._fail("NDJSON line deadline exceeded")
        source = bytes(data)
        offset = 0
        messages: list[dict[str, Any]] = []
        try:
            while offset < len(source):
                newline = source.find(b"\n", offset)
                end = len(source) if newline < 0 else newline + 1
                part = source[offset:end]
                if not self._buffer and part:
                    self._deadline = now + self.line_timeout_s
                if len(self._buffer) + len(part) > self.max_line_bytes:
                    self._fail("NDJSON line exceeds limit")
                self._buffer.extend(part)
                offset = end
                if newline >= 0:
                    messages.append(self._parse_line(bytes(self._buffer[:-1])))
                    self._buffer.clear()
                    self._deadline = None
            return messages
        except ProtocolError:
            raise

    def finish(self) -> None:
        if self._closed:
            raise ProtocolError("parser is closed")
        if self._buffer:
            self._fail("EOF in incomplete NDJSON line")
        self._closed = True

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

