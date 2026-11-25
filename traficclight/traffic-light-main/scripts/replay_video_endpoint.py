#!/usr/bin/env python3
"""One-connection P5/P6 replay sender or video receiver.

This is a staging endpoint: it exercises the frozen TLS and TLV1 path without
starting inference, a result service, systemd, or a production camera identity.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import socket
import sys
import threading
import time

from traffic_light_stream.config import loads_config
from traffic_light_stream.receiver import VideoReceiver
from traffic_light_stream.reconnect import ExponentialBackoff, connect_with_backoff
from traffic_light_stream.sender import ReplayFrameBuffer
from traffic_light_stream.source import ReplaySource
from traffic_light_stream.tls import loads_tls_config, wrap_client_socket, wrap_server_socket


def _load_stream(path: Path, role: str):
    config = loads_config(path.read_text(encoding="utf-8"))
    if config.role != role:
        raise ValueError(f"stream config role must be {role}")
    if not config.camera_id.startswith("replay:"):
        raise ValueError("staging endpoint requires a replay: camera identity")
    return config


def _load_tls(path: Path, side: str):
    return loads_tls_config(path.read_text(encoding="utf-8"), side=side)


def _association_update(digest, metadata, payload: bytes) -> None:
    digest.update(metadata["source_frame_seq"].to_bytes(8, "big"))
    digest.update(hashlib.sha256(payload).digest())


def run_server(args) -> dict:
    stream = _load_stream(args.stream_config, "service")
    tls = _load_tls(args.tls_config, "server")
    receiver = VideoReceiver(
        expected_camera_id=stream.camera_id,
        expected_source_mode="replay",
        max_metadata_bytes=stream.max_metadata_bytes,
        max_jpeg_bytes=stream.max_jpeg_bytes,
        max_width=stream.max_width,
        max_height=stream.max_height,
    )
    deadline = time.monotonic() + args.overall_timeout_s
    accepted_count = 0
    connection_records = []
    association = hashlib.sha256()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.host, stream.video_port))
        listener.listen(min(args.connection_count, 16))
        listener.settimeout(max(0.05, deadline - time.monotonic()))
        print(json.dumps({
            "status": "LISTENING", "host": args.host, "port": stream.video_port,
        }, sort_keys=True), flush=True)
        for connection_number in range(1, args.connection_count + 1):
            listener.settimeout(max(0.05, deadline - time.monotonic()))
            raw, peer = listener.accept()
            raw.settimeout(max(0.05, deadline - time.monotonic()))
            connection_association = hashlib.sha256()
            connection_count = 0
            last_sequence = 0
            source_boot_id = None
            sender_instance_id = None
            with wrap_server_socket(raw, tls) as connection:
                generation = receiver.begin_connection(peer_role=tls.expected_peer_role)
                try:
                    while True:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError("replay receiver overall deadline expired")
                        connection.settimeout(min(stream.video_idle_timeout_s, remaining))
                        data = connection.recv(64 * 1024)
                        if not data:
                            receiver.end_connection(generation=generation)
                            break
                        for frame in receiver.feed(data, generation=generation):
                            accepted_count += 1
                            connection_count += 1
                            last_sequence = frame.metadata["source_frame_seq"]
                            source_boot_id = frame.metadata["source_boot_id"]
                            sender_instance_id = frame.metadata["sender_instance_id"]
                            _association_update(association, frame.metadata, frame.jpeg)
                            _association_update(connection_association, frame.metadata, frame.jpeg)
                except BaseException:
                    if receiver.active_generation == generation:
                        receiver.abort_connection(generation=generation)
                    raise
                tls_version = connection.version()
                cipher = connection.cipher()[0]
            connection_records.append({
                "connection_number": connection_number,
                "connection_generation": generation,
                "accepted_frames": connection_count,
                "last_sequence": last_sequence,
                "source_boot_id": source_boot_id,
                "sender_instance_id": sender_instance_id,
                "association_sha256": connection_association.hexdigest(),
                "peer_ip": peer[0],
            })
    if args.expected_frames is not None and accepted_count != args.expected_frames:
        raise RuntimeError(
            f"received {accepted_count} frames, expected {args.expected_frames}"
        )
    return {
        "status": "PASS",
        "side": "server",
        "peer_ip": connection_records[-1]["peer_ip"],
        "tls_version": tls_version,
        "cipher": cipher,
        "accepted_frames": accepted_count,
        "last_sequence": last_sequence,
        "association_sha256": association.hexdigest(),
        "connection_count": len(connection_records),
        "connections": connection_records,
    }


def run_client(args) -> dict:
    stream = _load_stream(args.stream_config, "sender")
    tls = _load_tls(args.tls_config, "client")
    payloads = tuple(path.read_bytes() for path in args.jpeg)
    expanded = tuple(payloads[index % len(payloads)] for index in range(args.frame_count))
    source = ReplaySource(
        expanded,
        camera_id=stream.camera_id,
        width=args.width,
        height=args.height,
    )
    buffer = ReplayFrameBuffer(source)
    association = hashlib.sha256()
    connection_records = []
    backoff = ExponentialBackoff()
    deadline = time.monotonic() + args.overall_timeout_s
    for connection_number in range(1, args.connection_count + 1):
        def connect():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("replay sender overall deadline expired")
            return socket.create_connection(
                (stream.service_host, stream.video_port), timeout=max(0.05, remaining)
            )

        raw = connect_with_backoff(
            connect,
            max_attempts=args.max_connect_attempts,
            backoff=backoff,
            deadline=deadline,
        )
        raw.settimeout(max(0.05, deadline - time.monotonic()))
        connection_association = hashlib.sha256()
        producer_error = []
        producer_done = threading.Event()
        dropped_before = buffer.slot.dropped_count
        with wrap_client_socket(raw, tls) as connection:
            generation = buffer.begin_epoch(rewind=True)

            def produce() -> None:
                period = 1.0 / args.fps
                target = time.monotonic()
                try:
                    for _ in range(args.frame_count):
                        target += period
                        buffer.capture_one(generation=generation)
                        remaining = target - time.monotonic()
                        if remaining > 0:
                            time.sleep(remaining)
                except BaseException as exc:
                    producer_error.append(exc)
                finally:
                    producer_done.set()

            thread = threading.Thread(target=produce, name="replay-producer", daemon=True)
            thread.start()
            sent = 0
            last_sequence = 0
            while True:
                sequence = buffer.send_latest(
                    connection,
                    generation=generation,
                    max_age_s=stream.source_frame_age_limit_s,
                    write_timeout_s=stream.video_write_frame_timeout_s,
                    wait_timeout_s=min(0.05, stream.video_idle_timeout_s),
                )
                if sequence is None:
                    if producer_done.is_set():
                        break
                    continue
                frame_hash = hashlib.sha256(expanded[sequence - 1]).digest()
                association.update(sequence.to_bytes(8, "big"))
                association.update(frame_hash)
                connection_association.update(sequence.to_bytes(8, "big"))
                connection_association.update(frame_hash)
                sent += 1
                last_sequence = sequence
            thread.join(1)
            if thread.is_alive():
                raise RuntimeError("replay producer did not stop")
            if producer_error:
                raise producer_error[0]
            tls_version = connection.version()
            cipher = connection.cipher()[0]
        if sent != args.frame_count:
            raise RuntimeError(
                f"network consumed {sent} of {args.frame_count} frames; reduce replay fps"
            )
        connection_records.append({
            "connection_number": connection_number,
            "sender_generation": generation,
            "sent_frames": sent,
            "last_sequence": last_sequence,
            "source_boot_id": source.source_boot_id,
            "association_sha256": connection_association.hexdigest(),
            "producer_overwrites": buffer.slot.dropped_count - dropped_before,
        })
    return {
        "status": "PASS",
        "side": "client",
        "tls_version": tls_version,
        "cipher": cipher,
        "sent_frames": sum(item["sent_frames"] for item in connection_records),
        "last_sequence": last_sequence,
        "producer_overwrites": buffer.slot.dropped_count,
        "association_sha256": association.hexdigest(),
        "source_boot_id": source.source_boot_id,
        "sender_instance_id": source.sender_instance_id,
        "connection_count": len(connection_records),
        "connections": connection_records,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("side", choices=("server", "client"))
    parser.add_argument("--host", default="0.0.0.0", help="server bind address")
    parser.add_argument("--stream-config", required=True, type=Path)
    parser.add_argument("--tls-config", required=True, type=Path)
    parser.add_argument("--overall-timeout-s", type=float, default=15.0)
    parser.add_argument("--expected-frames", type=int)
    parser.add_argument("--connection-count", type=int, default=1)
    parser.add_argument("--max-connect-attempts", type=int, default=5)
    parser.add_argument("--jpeg", action="append", type=Path)
    parser.add_argument("--frame-count", type=int, default=30)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if (not 0.5 <= args.overall_timeout_s <= 120
            or not 1 <= args.connection_count <= 16
            or not 1 <= args.max_connect_attempts <= 100):
        parser.error("overall timeout must be 0.5..120 seconds")
    if args.side == "client":
        if (not args.jpeg or args.width is None or args.height is None
                or args.frame_count < 1 or not 0.1 <= args.fps <= 120):
            parser.error("client requires JPEG(s), dimensions, positive count, and fps 0.1..120")
        if any(not path.is_file() for path in args.jpeg):
            parser.error("every JPEG path must be a file")
    elif args.expected_frames is not None and args.expected_frames < 1:
        parser.error("expected frames must be positive")
    try:
        result = run_server(args) if args.side == "server" else run_client(args)
        print(json.dumps(result, sort_keys=True), flush=True)
        return 0
    except BaseException as exc:
        print(json.dumps({
            "status": "FAIL", "side": args.side,
            "error_type": type(exc).__name__, "error": str(exc),
        }, sort_keys=True), file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
