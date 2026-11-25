"""Restricted local source-status channel for the vehicle-side verifier."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import socket
import stat
import struct
import time
from typing import Any, Iterable, Mapping

from . import ProtocolError


SOURCE_STATUS_SCHEMA_VERSION = 1
SOURCE_STATUS_MAX_BYTES = 4096
SOURCE_STATUS_REQUEST = b"status\n"
_PEER_CREDENTIALS = struct.Struct("3i")


class SourceUnavailable(ProtocolError):
    """Raised when local source identity or freshness cannot be established."""


@dataclass(frozen=True)
class SourceStatus:
    schema_version: int
    type: str
    source_camera_id: str
    source_host_boot_id: str
    sender_instance_id: str
    source_boot_id: str
    latest_frame_seq: int
    latest_capture_monotonic_ns: int
    timestamp_basis: str
    source_mode: str
    healthy: bool
    reason: str
    generated_monotonic_ns: int


@dataclass(frozen=True)
class VerifiedSourceStatus:
    status: SourceStatus
    identity_changed: bool
    age_ns: int


_FIELDS = frozenset(SourceStatus.__dataclass_fields__)
_TEXT_FIELDS = (
    "source_camera_id", "source_host_boot_id", "sender_instance_id", "source_boot_id",
    "timestamp_basis",
)


def _text(value: Any, field: str, *, allow_empty: bool = False) -> str:
    minimum = 0 if allow_empty else 1
    if (not isinstance(value, str) or not minimum <= len(value) <= 128
            or (value and not value.isprintable())):
        raise ProtocolError(f"invalid {field}")
    return value


def source_status_from_mapping(payload: Mapping[str, Any]) -> SourceStatus:
    if not isinstance(payload, Mapping):
        raise ProtocolError("source status must be an object")
    keys = set(payload)
    if any(not isinstance(key, str) for key in keys):
        raise ProtocolError("source status keys must be strings")
    missing, unknown = _FIELDS - keys, keys - _FIELDS
    if missing:
        raise ProtocolError("missing source status fields: " + ", ".join(sorted(missing)))
    if unknown:
        raise ProtocolError("unknown source status fields: " + ", ".join(sorted(unknown)))
    values = dict(payload)
    if values["schema_version"] != SOURCE_STATUS_SCHEMA_VERSION or type(values["schema_version"]) is not int:
        raise ProtocolError("invalid source status schema_version")
    if values["type"] != "source_status":
        raise ProtocolError("invalid source status type")
    for field in _TEXT_FIELDS:
        values[field] = _text(values[field], field)
    if values["source_mode"] not in ("camera", "replay"):
        raise ProtocolError("invalid source_mode")
    if type(values["healthy"]) is not bool:
        raise ProtocolError("healthy must be boolean")
    values["reason"] = _text(values["reason"], "reason", allow_empty=True)
    for field in ("latest_frame_seq", "latest_capture_monotonic_ns"):
        value = values[field]
        if type(value) is not int or not 0 <= value < 1 << 64:
            raise ProtocolError(f"invalid {field}")
    generated = values["generated_monotonic_ns"]
    if type(generated) is not int or not 1 <= generated < 1 << 64:
        raise ProtocolError("invalid generated_monotonic_ns")
    if values["latest_capture_monotonic_ns"] > generated:
        raise ProtocolError("latest capture time is later than status generation")
    if values["healthy"]:
        if values["reason"]:
            raise ProtocolError("healthy source status cannot contain a failure reason")
        if values["latest_frame_seq"] == 0 or values["latest_capture_monotonic_ns"] == 0:
            raise ProtocolError("healthy source status requires a captured frame")
    elif not values["reason"]:
        raise ProtocolError("unhealthy source status requires a reason")
    return SourceStatus(**values)


def encode_source_status(status: SourceStatus) -> bytes:
    clean = source_status_from_mapping(asdict(status))
    encoded = json.dumps(asdict(clean), ensure_ascii=False, allow_nan=False,
                         separators=(",", ":")).encode("utf-8") + b"\n"
    if len(encoded) > SOURCE_STATUS_MAX_BYTES:
        raise ProtocolError("source status exceeds size limit")
    return encoded


def decode_source_status(payload: bytes) -> SourceStatus:
    if (not isinstance(payload, bytes) or not payload.endswith(b"\n")
            or payload.count(b"\n") != 1 or not 1 < len(payload) <= SOURCE_STATUS_MAX_BYTES):
        raise ProtocolError("invalid source status record boundary")

    def reject_duplicates(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ProtocolError(f"duplicate source status key: {key}")
            result[key] = value
        return result

    def reject_constant(value):
        raise ProtocolError(f"invalid JSON constant: {value}")

    try:
        decoded = json.loads(payload[:-1].decode("utf-8"),
                             object_pairs_hook=reject_duplicates,
                             parse_constant=reject_constant)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid source status JSON") from exc
    return source_status_from_mapping(decoded)


class SourceStatusTracker:
    """Verify repeated status snapshots against the same host monotonic clock."""

    def __init__(self, *, expected_camera_id: str, expected_host_boot_id: str,
                 expected_source_mode: str, max_age_s: float):
        self.expected_camera_id = _text(expected_camera_id, "expected_camera_id")
        self.expected_host_boot_id = _text(expected_host_boot_id, "expected_host_boot_id")
        if expected_source_mode not in ("camera", "replay"):
            raise ValueError("expected_source_mode must be camera or replay")
        if (isinstance(max_age_s, bool) or not isinstance(max_age_s, (int, float))
                or not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ValueError("max_age_s must be positive and finite")
        self.expected_source_mode = expected_source_mode
        self.max_age_ns = int(max_age_s * 1_000_000_000)
        self._identity: tuple[str, str] | None = None
        self._last_sequence = 0
        self._last_capture_ns = 0

    def accept(self, status: SourceStatus, *,
               receive_monotonic_ns: int | None = None) -> VerifiedSourceStatus:
        status = source_status_from_mapping(asdict(status))
        receive_ns = time.monotonic_ns() if receive_monotonic_ns is None else receive_monotonic_ns
        if type(receive_ns) is not int or receive_ns < status.generated_monotonic_ns:
            raise SourceUnavailable("source status time is in the future or invalid")
        age_ns = receive_ns - status.generated_monotonic_ns
        if age_ns > self.max_age_ns:
            raise SourceUnavailable("source status is stale")
        if (status.source_camera_id != self.expected_camera_id
                or status.source_host_boot_id != self.expected_host_boot_id
                or status.source_mode != self.expected_source_mode):
            raise SourceUnavailable("source status identity is not admitted")
        if not status.healthy:
            raise SourceUnavailable(f"source is unhealthy: {status.reason}")

        identity = (status.sender_instance_id, status.source_boot_id)
        changed = self._identity is not None and identity != self._identity
        if changed:
            self._last_sequence = 0
            self._last_capture_ns = 0
        elif (status.latest_frame_seq < self._last_sequence
              or status.latest_capture_monotonic_ns < self._last_capture_ns):
            raise SourceUnavailable("source status sequence or capture time regressed")
        self._identity = identity
        self._last_sequence = status.latest_frame_seq
        self._last_capture_ns = status.latest_capture_monotonic_ns
        return VerifiedSourceStatus(status, changed, age_ns)


def peer_credentials(connection: socket.socket) -> tuple[int, int, int]:
    if not hasattr(socket, "SO_PEERCRED"):
        raise RuntimeError("SO_PEERCRED is unavailable on this platform")
    raw = connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, _PEER_CREDENTIALS.size)
    return _PEER_CREDENTIALS.unpack(raw)


def bind_source_status_socket(path: str, *, mode: int = 0o660,
                              backlog: int = 4) -> socket.socket:
    """Bind a local socket without unlinking symlinks or unrelated files."""

    socket_path = Path(path)
    if not socket_path.is_absolute():
        raise ValueError("source status socket path must be absolute")
    if mode not in (0o600, 0o660):
        raise ValueError("source status socket mode must be 0600 or 0660")
    if socket_path.exists() or socket_path.is_symlink():
        info = socket_path.lstat()
        if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.geteuid():
            raise FileExistsError("refusing to replace non-owned source status path")
        socket_path.unlink()
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        listener.bind(os.fspath(socket_path))
        os.chmod(socket_path, mode, follow_symlinks=False)
        listener.listen(backlog)
        return listener
    except BaseException:
        listener.close()
        try:
            socket_path.unlink()
        except OSError:
            pass
        raise


def serve_source_status_once(listener: socket.socket, status: SourceStatus, *,
                             allowed_uids: Iterable[int], timeout_s: float = 0.5) -> bool:
    allowed = frozenset(allowed_uids)
    if not allowed or any(type(uid) is not int or uid < 0 for uid in allowed):
        raise ValueError("allowed_uids must contain non-negative integers")
    listener.settimeout(timeout_s)
    connection, _ = listener.accept()
    with connection:
        connection.settimeout(timeout_s)
        _, uid, _ = peer_credentials(connection)
        if uid not in allowed:
            return False
        request = bytearray()
        while b"\n" not in request and len(request) <= len(SOURCE_STATUS_REQUEST):
            chunk = connection.recv(len(SOURCE_STATUS_REQUEST) + 1 - len(request))
            if not chunk:
                break
            request.extend(chunk)
        if bytes(request) != SOURCE_STATUS_REQUEST:
            raise ProtocolError("invalid source status request")
        connection.sendall(encode_source_status(status))
        return True


def query_source_status(path: str, *, timeout_s: float = 0.5) -> SourceStatus:
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout_s)
            connection.connect(path)
            connection.sendall(SOURCE_STATUS_REQUEST)
            payload = bytearray()
            while b"\n" not in payload:
                chunk = connection.recv(SOURCE_STATUS_MAX_BYTES + 1 - len(payload))
                if not chunk:
                    raise SourceUnavailable("source status server closed without a record")
                payload.extend(chunk)
                if len(payload) > SOURCE_STATUS_MAX_BYTES:
                    raise ProtocolError("source status exceeds size limit")
            return decode_source_status(bytes(payload))
    except SourceUnavailable:
        raise
    except (OSError, TimeoutError) as exc:
        raise SourceUnavailable("cannot obtain source status") from exc
