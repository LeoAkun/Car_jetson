"""Fail-closed TLS 1.3 mutual-authentication helpers for P5/P6."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import select
import socket
import ssl
import stat
import time
from typing import Any, Mapping


TLS_ROLE_URI_PREFIX = "urn:car-jetson:traffic-light:role:"
TLS_ROLES = frozenset(("service", "video-sender", "controller"))


class TLSConfigurationError(ValueError):
    """Raised before networking when TLS configuration is unsafe or invalid."""


class TLSRoleError(ssl.SSLError):
    """Raised after certificate validation when the application role is wrong."""


class TLSHandshakeTimeout(TimeoutError):
    """Raised when the one total TLS handshake deadline expires."""


@dataclass(frozen=True)
class TLSConfig:
    """Secret-free TLS file locations and peer authorization policy.

    ``server_name`` is required on clients and must be ``None`` on servers.
    A service uses one server-side instance per endpoint because the video and
    result endpoints authorize different client roles.
    """

    ca_file: str
    cert_file: str
    key_file: str
    server_name: str | None
    expected_peer_role: str
    handshake_timeout_s: float


_TLS_CONFIG_FIELDS = frozenset(TLSConfig.__dataclass_fields__)


def _absolute_file_path(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value or not value.isprintable():
        raise TLSConfigurationError(f"invalid {field}")
    path = Path(value)
    if not path.is_absolute():
        raise TLSConfigurationError(f"{field} must be an absolute path")
    return os.fspath(path)


def tls_config_from_mapping(payload: Mapping[str, Any], *, side: str) -> TLSConfig:
    """Parse an exact TLS configuration object for ``client`` or ``server``."""

    if side not in ("client", "server"):
        raise TLSConfigurationError("side must be client or server")
    if not isinstance(payload, Mapping):
        raise TLSConfigurationError("TLS configuration must be an object")
    keys = set(payload)
    if any(not isinstance(key, str) for key in keys):
        raise TLSConfigurationError("TLS configuration keys must be strings")
    missing, unknown = _TLS_CONFIG_FIELDS - keys, keys - _TLS_CONFIG_FIELDS
    if missing:
        raise TLSConfigurationError("missing TLS config fields: " + ", ".join(sorted(missing)))
    if unknown:
        raise TLSConfigurationError("unknown TLS config fields: " + ", ".join(sorted(unknown)))

    values = dict(payload)
    for field in ("ca_file", "cert_file", "key_file"):
        values[field] = _absolute_file_path(values[field], field)

    server_name = values["server_name"]
    if side == "client":
        if (not isinstance(server_name, str) or not server_name
                or len(server_name) > 253 or not server_name.isprintable()):
            raise TLSConfigurationError("client server_name must be printable text")
    elif server_name is not None:
        raise TLSConfigurationError("server server_name must be null")

    role = values["expected_peer_role"]
    if role not in TLS_ROLES:
        raise TLSConfigurationError("invalid expected_peer_role")

    timeout = values["handshake_timeout_s"]
    if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout) or not 0.05 <= timeout <= 30.0):
        raise TLSConfigurationError("invalid handshake_timeout_s")
    values["handshake_timeout_s"] = float(timeout)
    return TLSConfig(**values)


def loads_tls_config(text: str, *, side: str) -> TLSConfig:
    """Load strict JSON while rejecting duplicate keys and non-finite values."""

    if not isinstance(text, str):
        raise TLSConfigurationError("TLS configuration must be UTF-8 text")

    def reject_constant(value: str) -> None:
        raise TLSConfigurationError(f"invalid JSON constant: {value}")

    def reject_duplicates(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise TLSConfigurationError(f"duplicate TLS config key: {key}")
            result[key] = value
        return result

    try:
        payload = json.loads(
            text,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicates,
        )
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise TLSConfigurationError("invalid TLS config JSON") from exc
    return tls_config_from_mapping(payload, side=side)


def _check_regular_file(path: str, field: str) -> None:
    try:
        info = os.stat(path, follow_symlinks=False)
    except OSError as exc:
        raise TLSConfigurationError(f"cannot access {field}") from exc
    if not stat.S_ISREG(info.st_mode):
        raise TLSConfigurationError(f"{field} must be a regular file")


def _check_credentials(config: TLSConfig) -> None:
    for field in ("ca_file", "cert_file", "key_file"):
        if not Path(getattr(config, field)).is_absolute():
            raise TLSConfigurationError(f"{field} must be an absolute path")
    _check_regular_file(config.ca_file, "ca_file")
    _check_regular_file(config.cert_file, "cert_file")
    _check_regular_file(config.key_file, "key_file")
    mode = stat.S_IMODE(os.stat(config.key_file, follow_symlinks=False).st_mode)
    if mode not in (0o400, 0o600):
        raise TLSConfigurationError("key_file mode must be 0400 or 0600")


def _restrict_to_tls13(context: ssl.SSLContext) -> None:
    if not ssl.HAS_TLSv1_3:
        raise TLSConfigurationError("Python/OpenSSL has no TLS 1.3 support")
    context.minimum_version = ssl.TLSVersion.TLSv1_3
    context.maximum_version = ssl.TLSVersion.TLSv1_3
    context.options |= ssl.OP_NO_COMPRESSION


def create_client_context(config: TLSConfig) -> ssl.SSLContext:
    """Build a TLS 1.3 client context with CA, hostname and client auth."""

    if config.server_name is None:
        raise TLSConfigurationError("client server_name is required")
    _check_credentials(config)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    _restrict_to_tls13(context)
    context.check_hostname = True
    context.verify_mode = ssl.CERT_REQUIRED
    try:
        context.load_verify_locations(cafile=config.ca_file)
        context.load_cert_chain(config.cert_file, config.key_file)
    except (OSError, ssl.SSLError) as exc:
        raise TLSConfigurationError("cannot load client TLS credentials") from exc
    return context


def create_server_context(config: TLSConfig) -> ssl.SSLContext:
    """Build a TLS 1.3 server context that requires a client certificate."""

    if config.server_name is not None:
        raise TLSConfigurationError("server server_name must be null")
    _check_credentials(config)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    _restrict_to_tls13(context)
    context.check_hostname = False
    context.verify_mode = ssl.CERT_REQUIRED
    try:
        context.load_verify_locations(cafile=config.ca_file)
        context.load_cert_chain(config.cert_file, config.key_file)
    except (OSError, ssl.SSLError) as exc:
        raise TLSConfigurationError("cannot load server TLS credentials") from exc
    return context


def peer_application_role(peer_certificate: Mapping[str, Any]) -> str:
    """Return the one recognized application role encoded as a URI SAN."""

    if not peer_certificate:
        raise TLSRoleError("peer certificate is unavailable")
    roles = []
    for kind, value in peer_certificate.get("subjectAltName", ()):
        if kind != "URI" or not isinstance(value, str):
            continue
        if value.startswith(TLS_ROLE_URI_PREFIX):
            roles.append(value[len(TLS_ROLE_URI_PREFIX):])
    if len(roles) != 1 or roles[0] not in TLS_ROLES:
        raise TLSRoleError("peer certificate must contain exactly one recognized role URI SAN")
    return roles[0]


def verify_peer_role(tls_socket: ssl.SSLSocket, expected_role: str) -> str:
    """Authorize the TLS peer for one P5/P6 application role."""

    if expected_role not in TLS_ROLES:
        raise TLSConfigurationError("invalid expected peer role")
    actual_role = peer_application_role(tls_socket.getpeercert())
    if actual_role != expected_role:
        raise TLSRoleError(
            f"peer application role {actual_role!r} is not authorized as {expected_role!r}"
        )
    return actual_role


def _wait_for_handshake_io(
    tls_socket: ssl.SSLSocket,
    *,
    want_read: bool,
    deadline: float,
    clock,
) -> None:
    remaining = deadline - clock()
    if remaining <= 0:
        raise TLSHandshakeTimeout("TLS handshake deadline expired")
    read_list = [tls_socket] if want_read else []
    write_list = [] if want_read else [tls_socket]
    readable, writable, _ = select.select(read_list, write_list, [], remaining)
    if not readable and not writable:
        raise TLSHandshakeTimeout("TLS handshake deadline expired")


def complete_handshake(
    tls_socket: ssl.SSLSocket,
    *,
    expected_peer_role: str,
    timeout_s: float,
    clock=time.monotonic,
) -> ssl.SSLSocket:
    """Complete and authorize one handshake under a single total deadline.

    The deadline starts once this function is entered. Progress does not reset
    it. Any TLS, timeout, role or I/O failure closes the TLS socket.
    """

    if expected_peer_role not in TLS_ROLES:
        tls_socket.close()
        raise TLSConfigurationError("invalid expected peer role")
    if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
            or not math.isfinite(timeout_s) or timeout_s <= 0):
        tls_socket.close()
        raise TLSConfigurationError("invalid TLS handshake timeout")

    deadline = clock() + float(timeout_s)
    previous_timeout = tls_socket.gettimeout()
    tls_socket.setblocking(False)
    try:
        while True:
            if clock() >= deadline:
                raise TLSHandshakeTimeout("TLS handshake deadline expired")
            try:
                tls_socket.do_handshake()
                break
            except ssl.SSLWantReadError:
                _wait_for_handshake_io(
                    tls_socket, want_read=True, deadline=deadline, clock=clock
                )
            except ssl.SSLWantWriteError:
                _wait_for_handshake_io(
                    tls_socket, want_read=False, deadline=deadline, clock=clock
                )
        verify_peer_role(tls_socket, expected_peer_role)
        tls_socket.settimeout(previous_timeout)
        return tls_socket
    except BaseException:
        tls_socket.close()
        raise


def wrap_client_socket(
    raw_socket: socket.socket,
    config: TLSConfig,
) -> ssl.SSLSocket:
    """Wrap, handshake and authorize one already-connected client socket."""

    if config.server_name is None:
        raw_socket.close()
        raise TLSConfigurationError("client server_name is required")
    try:
        context = create_client_context(config)
        tls_socket = context.wrap_socket(
            raw_socket,
            server_hostname=config.server_name,
            do_handshake_on_connect=False,
        )
    except BaseException:
        raw_socket.close()
        raise
    return complete_handshake(
        tls_socket,
        expected_peer_role=config.expected_peer_role,
        timeout_s=config.handshake_timeout_s,
    )


def wrap_server_socket(
    raw_socket: socket.socket,
    config: TLSConfig,
) -> ssl.SSLSocket:
    """Wrap, handshake and authorize one accepted server-side socket."""

    if config.server_name is not None:
        raw_socket.close()
        raise TLSConfigurationError("server server_name must be null")
    try:
        context = create_server_context(config)
        tls_socket = context.wrap_socket(
            raw_socket,
            server_side=True,
            do_handshake_on_connect=False,
        )
    except BaseException:
        raw_socket.close()
        raise
    return complete_handshake(
        tls_socket,
        expected_peer_role=config.expected_peer_role,
        timeout_s=config.handshake_timeout_s,
    )
