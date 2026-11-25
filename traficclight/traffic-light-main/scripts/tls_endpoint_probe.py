#!/usr/bin/env python3
"""One-connection P5/P6 TLS 1.3 device probe with no business protocol."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import time

from traffic_light_stream.tls import (
    loads_tls_config,
    wrap_client_socket,
    wrap_server_socket,
)


PROBE_REQUEST = b"traffic-light-mtls-probe-v1\n"
PROBE_RESPONSE = b"traffic-light-mtls-ready-v1\n"


def load_config(path: Path, side: str):
    return loads_tls_config(path.read_text(encoding="utf-8"), side=side)


def report(side: str, tls_socket: socket.socket, elapsed_s: float) -> None:
    peer = tls_socket.getpeercert()
    print(json.dumps({
        "status": "PASS",
        "side": side,
        "tls_version": tls_socket.version(),
        "cipher": tls_socket.cipher()[0],
        "peer_subject": peer.get("subject"),
        "peer_san": peer.get("subjectAltName"),
        "elapsed_ms": round(elapsed_s * 1000, 3),
    }, sort_keys=True))


def run_server(args) -> None:
    config = load_config(args.config, "server")
    deadline = time.monotonic() + args.overall_timeout_s
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.host, args.port))
        listener.listen(1)
        listener.settimeout(max(0.05, deadline - time.monotonic()))
        print(json.dumps({"status": "LISTENING", "host": args.host, "port": args.port}), flush=True)
        raw, peer = listener.accept()
        raw.settimeout(max(0.05, deadline - time.monotonic()))
        started = time.monotonic()
        with wrap_server_socket(raw, config) as tls_socket:
            payload = tls_socket.recv(len(PROBE_REQUEST) + 1)
            if payload != PROBE_REQUEST:
                raise RuntimeError(f"invalid probe payload from {peer[0]}")
            tls_socket.sendall(PROBE_RESPONSE)
            report("server", tls_socket, time.monotonic() - started)


def run_client(args) -> None:
    config = load_config(args.config, "client")
    started = time.monotonic()
    raw = socket.create_connection((args.host, args.port), timeout=args.overall_timeout_s)
    raw.settimeout(args.overall_timeout_s)
    with wrap_client_socket(raw, config) as tls_socket:
        tls_socket.sendall(PROBE_REQUEST)
        payload = tls_socket.recv(len(PROBE_RESPONSE) + 1)
        if payload != PROBE_RESPONSE:
            raise RuntimeError("invalid probe response")
        report("client", tls_socket, time.monotonic() - started)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("side", choices=("server", "client"))
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--overall-timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not 0.5 <= args.overall_timeout_s <= 30.0:
        parser.error("invalid port or timeout")
    try:
        if args.side == "server":
            run_server(args)
        else:
            run_client(args)
    except BaseException as exc:
        print(json.dumps({
            "status": "FAIL",
            "side": args.side,
            "error_type": type(exc).__name__,
            "error": str(exc),
        }, sort_keys=True), file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
