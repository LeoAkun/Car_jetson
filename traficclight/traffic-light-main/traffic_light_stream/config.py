"""Strict, secret-free P5/P6 runtime configuration."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import ipaddress
import json
import math
import re
from typing import Any, Mapping

from . import ProtocolError


@dataclass(frozen=True)
class StreamConfig:
    role: str
    service_host: str
    camera_id: str
    video_port: int
    result_port: int
    max_metadata_bytes: int
    max_jpeg_bytes: int
    max_result_line_bytes: int
    max_width: int
    max_height: int
    tls_handshake_timeout_s: float
    video_read_frame_timeout_s: float
    video_write_frame_timeout_s: float
    video_idle_timeout_s: float
    inference_timeout_s: float
    observation_timeout_s: float
    result_read_line_timeout_s: float
    result_write_timeout_s: float
    heartbeat_interval_s: float
    heartbeat_timeout_s: float
    ack_timeout_s: float
    session_lease_s: float
    session_renew_interval_s: float
    target_inference_interval_s: float
    source_frame_age_limit_s: float
    result_ttl_s: float


_FIELDS = set(StreamConfig.__dataclass_fields__)
_INTEGER_RANGES = {
    "video_port": (1, 65535), "result_port": (1, 65535),
    "max_metadata_bytes": (1, 4096), "max_jpeg_bytes": (1, 4 * 1024 * 1024),
    "max_result_line_bytes": (2, 65536), "max_width": (1, 1920), "max_height": (1, 1080),
}
_DURATION_FIELDS = _FIELDS - {"role", "service_host", "camera_id"} - set(_INTEGER_RANGES)
_DURATION_BOUNDS = (0.001, 120.0)


def _strict_text(value: Any, field: str, maximum: int = 128) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= maximum or not value.isprintable():
        raise ProtocolError(f"invalid {field}")
    return value


def _validate_host(host: Any) -> str:
    host = _strict_text(host, "service_host", 253)
    try:
        ipaddress.ip_address(host)
        return host
    except ValueError:
        if (host.startswith(".") or host.endswith(".") or ".." in host
                or not re.fullmatch(r"[A-Za-z0-9.-]+", host)
                or any(not part or len(part) > 63 or part.startswith("-") or part.endswith("-")
                       for part in host.split("."))):
            raise ProtocolError("invalid service_host")
        return host


def from_mapping(payload: Mapping[str, Any]) -> StreamConfig:
    if not isinstance(payload, Mapping):
        raise ProtocolError("configuration must be an object")
    keys = set(payload)
    if any(not isinstance(key, str) for key in keys):
        raise ProtocolError("configuration keys must be strings")
    missing, unknown = _FIELDS - keys, keys - _FIELDS
    if missing:
        raise ProtocolError("missing config fields: " + ", ".join(sorted(missing)))
    if unknown:
        raise ProtocolError("unknown config fields: " + ", ".join(sorted(unknown)))
    values = dict(payload)
    if values["role"] not in ("sender", "service", "controller"):
        raise ProtocolError("role must be sender, service, or controller")
    values["service_host"] = _validate_host(values["service_host"])
    values["camera_id"] = _strict_text(values["camera_id"], "camera_id")
    for field, (minimum, maximum) in _INTEGER_RANGES.items():
        value = values[field]
        if type(value) is not int or not minimum <= value <= maximum:
            raise ProtocolError(f"invalid {field}")
    for field in _DURATION_FIELDS:
        value = values[field]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError(f"invalid {field}")
        try:
            valid_duration = math.isfinite(value) and _DURATION_BOUNDS[0] <= value <= _DURATION_BOUNDS[1]
        except OverflowError:
            valid_duration = False
        if not valid_duration:
            raise ProtocolError(f"invalid {field}")
        values[field] = float(value)
    if values["video_port"] == values["result_port"]:
        raise ProtocolError("video_port and result_port must differ")
    if values["heartbeat_interval_s"] >= values["heartbeat_timeout_s"]:
        raise ProtocolError("heartbeat timeout must exceed interval")
    if values["session_renew_interval_s"] >= values["session_lease_s"]:
        raise ProtocolError("session lease must exceed renew interval")
    if values["ack_timeout_s"] < values["result_read_line_timeout_s"]:
        raise ProtocolError("ack timeout must cover a complete result line")
    try:
        return StreamConfig(**values)
    except TypeError as exc:
        raise ProtocolError("invalid configuration shape") from exc


def loads_config(text: str) -> StreamConfig:
    if not isinstance(text, str):
        raise ProtocolError("configuration must be UTF-8 text")

    def reject_constant(value):
        raise ProtocolError(f"invalid JSON constant: {value}")

    def reject_duplicates(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ProtocolError(f"duplicate config key: {key}")
            result[key] = value
        return result

    try:
        payload = json.loads(text, parse_constant=reject_constant, object_pairs_hook=reject_duplicates)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ProtocolError("invalid config JSON") from exc
    return from_mapping(payload)


def summary(config: StreamConfig) -> dict[str, Any]:
    """Return operational settings without any secret-bearing fields."""
    return asdict(config)

