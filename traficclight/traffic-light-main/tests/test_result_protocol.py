import unittest

from traffic_light_stream import ProtocolError
from traffic_light_stream.result_protocol import (
    NDJSONParser, encode_message, validate_message,
)


def hello():
    return {
        "schema_version": 2, "type": "hello", "role": "controller",
        "peer_id": "vehicle-controller", "peer_boot_id": "host-boot-uuid",
        "capabilities": ["schema2", "source-age-check"],
    }


def arm():
    return {
        "schema_version": 2, "type": "command", "action": "arm", "request_id": "req-001",
        "session_id": "session-001", "task_id": "shadow-test-001", "map_name": "K8",
        "waypoint_id": 42, "direction": "straight", "roi_norm": [0.30, 0.05, 0.72, 0.58],
        "expected_model_sha256": "a" * 64, "expected_detector_boot_id": "detector-boot-uuid",
        "source_camera_id": "front-traffic-port", "source_host_boot_id": "host-boot-uuid",
        "source_boot_id": "stream-epoch-uuid", "arm_monotonic_ns": 1234567890000,
    }


def observation():
    return {
        "schema_version": 2, "type": "observation", "service_boot_id": "service-boot-uuid",
        "seq": 1, "session_id": "session-001", "task_id": "shadow-test-001", "map_name": "K8",
        "waypoint_id": 42, "direction": "straight", "model_sha256": "a" * 64,
        "roi_norm": [0.30, 0.05, 0.72, 0.58], "source_camera_id": "front-traffic-port",
        "source_host_boot_id": "host-boot-uuid", "source_boot_id": "stream-epoch-uuid",
        "source_frame_seq": 9271, "source_capture_monotonic_ns": 1234567890123,
        "timestamp_basis": "v4l2_monotonic_eof", "source_mode": "camera",
        "receive_monotonic_ns": 2234567890123, "state": "UNKNOWN", "valid": False,
        "confidence": 0.0, "has_bbox": False, "bbox_norm": [0, 0, 0, 0],
        "reason": "no_eligible_candidate", "inference_ms": 19.01,
        "control_eligible": False, "automatic_release_enabled": False,
        "thresholds_calibrated": False,
    }


def heartbeat():
    return {
        "schema_version": 2, "type": "heartbeat", "service_boot_id": "service-boot-uuid",
        "heartbeat_seq": 1, "ready": False, "armed": False, "session_id": None,
        "camera_ok": False, "model_ok": True, "model_validation_status": "candidate",
        "control_eligible": False, "automatic_release_enabled": False,
        "thresholds_calibrated": False, "source_camera_id": "front-traffic-port",
        "source_host_boot_id": None, "source_boot_id": None, "receive_fps": 0.0,
        "inference_fps": 0.0, "send_fps": 5.0, "source_queue_drops": 0,
        "result_queue_drops": 0, "queue_depth": 0, "last_frame_age_ms": None,
        "reason": "video_disconnected",
    }


class ResultProtocolTest(unittest.TestCase):
    def test_frozen_message_fixtures_validate(self):
        messages = [
            hello(), {"schema_version": 2, "type": "status", "request_id": "req-status"},
            arm(), {"schema_version": 2, "type": "command", "action": "renew",
                    "request_id": "req-renew", "session_id": "session-001"},
            {"schema_version": 2, "type": "command", "action": "disarm",
             "request_id": "req-disarm", "session_id": "session-001"},
            {"schema_version": 2, "type": "ack", "action": "arm", "request_id": "req-001",
             "accepted": True, "reason": "", "service_boot_id": "service-boot-uuid",
             "session_id": "session-001", "lease_duration_ms": 1500},
            observation(), heartbeat(),
            {"schema_version": 2, "type": "error", "code": "not_ready",
             "message": "source is unavailable", "request_id": "req-001"},
        ]
        for message in messages:
            self.assertEqual(validate_message(message), message)
            self.assertTrue(encode_message(message).endswith(b"\n"))

    def test_partial_and_coalesced_lines(self):
        first = encode_message(hello())
        second = encode_message({"schema_version": 2, "type": "status", "request_id": "查询"})
        parser = NDJSONParser()
        self.assertEqual(parser.feed(first[:9]), [])
        received = parser.feed(first[9:] + second)
        self.assertEqual([item["type"] for item in received], ["hello", "status"])
        self.assertEqual(received[1]["request_id"], "查询")
        parser.finish()

    def test_utf8_can_be_split_at_any_byte(self):
        line = encode_message({"schema_version": 2, "type": "error", "code": "source", "message": "道路不可用"})
        expected = [{"schema_version": 2, "type": "error", "code": "source", "message": "道路不可用"}]
        for cut in range(len(line) + 1):
            parser = NDJSONParser()
            received = parser.feed(line[:cut]) + parser.feed(line[cut:])
            self.assertEqual(received, expected, cut)

    def test_line_limit_counts_newline_and_is_inclusive(self):
        base = {"schema_version": 2, "type": "error", "code": "x", "message": "x"}
        base_length = len(encode_message(base))
        exact = {**base, "message": "a" * (257 - base_length)}
        line = encode_message(exact, max_line_bytes=256)
        self.assertEqual(len(line), 256)
        self.assertEqual(NDJSONParser(max_line_bytes=256).feed(line)[0], exact)
        oversized = {**exact, "message": exact["message"] + "a"}
        with self.assertRaises(ProtocolError):
            encode_message(oversized, max_line_bytes=256)
        parser = NDJSONParser(max_line_bytes=256)
        with self.assertRaises(ProtocolError):
            parser.feed(line[:-1] + b"aa")

    def test_bad_schema_duplicate_key_and_nonfinite_values_close_parser(self):
        for line in (
                b'{"schema_version":3,"type":"status","request_id":"x"}\n',
                b'{"schema_version":2,"schema_version":2,"type":"status","request_id":"x"}\n',
                b'{"schema_version":2,"type":"error","code":"x","message":NaN}\n'):
            parser = NDJSONParser()
            with self.assertRaises(ProtocolError):
                parser.feed(line)
            with self.assertRaises(ProtocolError):
                parser.feed(encode_message(hello()))

    def test_wrong_types_unknown_fields_and_candidate_flags_rejected(self):
        bad_messages = [
            {"schema_version": 2, "type": "status", "request_id": "x", "extra": 1},
            {"schema_version": 2, "type": "command", "action": "arm", "request_id": "x",
             **{key: value for key, value in arm().items() if key not in {"schema_version", "type", "action", "request_id"}},
             "waypoint_id": True},
            {**observation(), "control_eligible": True},
            {**observation(), "source_frame_seq": True},
            {**arm(), "roi_norm": [0.8, 0.2, 0.1, 0.7]},
            {**observation(), "confidence": 10 ** 1000},
            {**heartbeat(), "ready": True},
            {**heartbeat(), "armed": True},
            {"schema_version": 2, "type": "ack", "action": "unknown", "request_id": "x",
             "accepted": False, "reason": "rejected", "service_boot_id": "service-boot"},
        ]
        for message in bad_messages:
            with self.assertRaises(ProtocolError):
                validate_message(message)

    def test_nested_ack_bindings_reject_nonfinite_numbers(self):
        message = {"schema_version": 2, "type": "ack", "action": "arm", "request_id": "x",
                   "accepted": True, "reason": "", "service_boot_id": "service-boot",
                   "bindings": {"roi_norm": [0.1, float("inf"), 0.8, 0.9]}}
        with self.assertRaises(ProtocolError):
            validate_message(message)

    def test_incomplete_eof_and_total_line_deadline(self):
        now = [10.0]
        parser = NDJSONParser(line_timeout_s=0.5, clock=lambda: now[0])
        parser.feed(encode_message(hello())[:-1])
        now[0] = 10.5
        with self.assertRaises(ProtocolError):
            parser.feed(b"\n")

        parser = NDJSONParser()
        parser.feed(encode_message(hello())[:-1])
        with self.assertRaises(ProtocolError):
            parser.finish()

        with self.assertRaises(ValueError):
            NDJSONParser(line_timeout_s=10 ** 1000)


if __name__ == "__main__":
    unittest.main()

