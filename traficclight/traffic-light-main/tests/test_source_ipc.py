import json
import os
from pathlib import Path
import socket
import stat
import tempfile
import threading
import unittest

from traffic_light_stream import ProtocolError
from traffic_light_stream.source_ipc import (
    SourceStatus, SourceStatusTracker, SourceUnavailable,
    bind_source_status_socket, decode_source_status, encode_source_status,
    query_source_status, serve_source_status_once,
)


def status(**updates):
    values = {
        "schema_version": 1,
        "type": "source_status",
        "source_camera_id": "replay:front-traffic-port",
        "source_host_boot_id": "host-boot",
        "sender_instance_id": "sender-instance",
        "source_boot_id": "source-epoch",
        "latest_frame_seq": 3,
        "latest_capture_monotonic_ns": 900,
        "timestamp_basis": "replay_monotonic_generated",
        "source_mode": "replay",
        "healthy": True,
        "reason": "",
        "generated_monotonic_ns": 1000,
    }
    values.update(updates)
    return SourceStatus(**values)


class SourceStatusCodecTest(unittest.TestCase):
    def test_roundtrip_is_one_bounded_record(self):
        encoded = encode_source_status(status())
        self.assertTrue(encoded.endswith(b"\n"))
        self.assertEqual(decode_source_status(encoded), status())

    def test_duplicate_unknown_nonfinite_and_health_invariants_fail(self):
        valid = json.loads(encode_source_status(status()))
        duplicate = encode_source_status(status()).decode().rstrip("\n}") + ',"healthy":true}\n'
        records = [
            duplicate.encode(),
            (json.dumps({**valid, "extra": 1}) + "\n").encode(),
            (json.dumps({**valid, "generated_monotonic_ns": float("nan")}) + "\n").encode(),
            encode_source_status(status()).replace(b'"latest_frame_seq":3', b'"latest_frame_seq":true'),
        ]
        for record in records:
            with self.assertRaises(ProtocolError):
                decode_source_status(record)
        for invalid in (
            status(healthy=True, reason="failed"),
            status(healthy=False, reason=""),
            status(latest_capture_monotonic_ns=1001),
        ):
            with self.assertRaises(ProtocolError):
                encode_source_status(invalid)


class SourceStatusTrackerTest(unittest.TestCase):
    def tracker(self, max_age_s=0.000001):
        return SourceStatusTracker(
            expected_camera_id="replay:front-traffic-port",
            expected_host_boot_id="host-boot",
            expected_source_mode="replay",
            max_age_s=max_age_s,
        )

    def test_age_boundary_and_identity_change(self):
        tracker = self.tracker()
        first = tracker.accept(status(), receive_monotonic_ns=2000)
        self.assertEqual(first.age_ns, 1000)
        self.assertFalse(first.identity_changed)
        changed = tracker.accept(
            status(sender_instance_id="new-sender", source_boot_id="new-epoch",
                   latest_frame_seq=1),
            receive_monotonic_ns=2000,
        )
        self.assertTrue(changed.identity_changed)
        with self.assertRaises(SourceUnavailable):
            tracker.accept(status(generated_monotonic_ns=1000), receive_monotonic_ns=2001)

    def test_future_unhealthy_wrong_identity_and_regression_fail(self):
        cases = (
            (status(), 999),
            (status(healthy=False, reason="camera_stalled"), 1000),
            (status(source_host_boot_id="old-boot"), 1000),
        )
        for current, receive in cases:
            with self.assertRaises(SourceUnavailable):
                self.tracker().accept(current, receive_monotonic_ns=receive)
        tracker = self.tracker()
        tracker.accept(status(), receive_monotonic_ns=1000)
        with self.assertRaises(SourceUnavailable):
            tracker.accept(status(latest_frame_seq=2), receive_monotonic_ns=1000)


@unittest.skipUnless(hasattr(socket, "SO_PEERCRED"), "Linux peer credentials required")
class SourceStatusSocketTest(unittest.TestCase):
    def test_owned_socket_mode_peer_uid_and_query(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "source.sock")
            listener = bind_source_status_socket(path, mode=0o600)
            result = []

            def serve():
                result.append(serve_source_status_once(
                    listener, status(), allowed_uids={os.geteuid()}
                ))

            thread = threading.Thread(target=serve)
            thread.start()
            self.assertEqual(query_source_status(path), status())
            thread.join(1)
            listener.close()
            self.assertEqual(result, [True])
            self.assertEqual(stat.S_IMODE(os.lstat(path).st_mode), 0o600)

    def test_unauthorized_uid_receives_no_status(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "source.sock")
            listener = bind_source_status_socket(path)
            result = []
            thread = threading.Thread(target=lambda: result.append(
                serve_source_status_once(listener, status(), allowed_uids={os.geteuid() + 1})
            ))
            thread.start()
            with self.assertRaises(SourceUnavailable):
                query_source_status(path)
            thread.join(1)
            listener.close()
            self.assertEqual(result, [False])

    def test_split_stream_request_is_reassembled(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "source.sock")
            listener = bind_source_status_socket(path)
            thread = threading.Thread(target=lambda: serve_source_status_once(
                listener, status(), allowed_uids={os.geteuid()}
            ))
            thread.start()
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(path)
                client.sendall(b"sta")
                client.sendall(b"tus\n")
                payload = client.recv(4096)
            thread.join(1)
            listener.close()
            self.assertEqual(decode_source_status(payload), status())

    def test_refuses_to_replace_regular_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "source.sock"
            path.write_text("do not replace")
            with self.assertRaises(FileExistsError):
                bind_source_status_socket(str(path))
            self.assertEqual(path.read_text(), "do not replace")


if __name__ == "__main__":
    unittest.main()
