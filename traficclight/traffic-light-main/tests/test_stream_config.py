import json
import unittest

from traffic_light_stream import ProtocolError
from traffic_light_stream.config import StreamConfig, from_mapping, loads_config, summary


def config_values():
    return {
        "role": "sender", "service_host": "192.168.1.150", "camera_id": "front-traffic-port",
        "video_port": 5000, "result_port": 12345, "max_metadata_bytes": 4096,
        "max_jpeg_bytes": 4 * 1024 * 1024, "max_result_line_bytes": 65536,
        "max_width": 1920, "max_height": 1080, "tls_handshake_timeout_s": 3.0,
        "video_read_frame_timeout_s": 0.5, "video_write_frame_timeout_s": 0.2,
        "video_idle_timeout_s": 0.5, "inference_timeout_s": 0.25,
        "observation_timeout_s": 0.5, "result_read_line_timeout_s": 0.5,
        "result_write_timeout_s": 0.2, "heartbeat_interval_s": 0.2,
        "heartbeat_timeout_s": 0.6, "ack_timeout_s": 1.0, "session_lease_s": 1.5,
        "session_renew_interval_s": 0.5, "target_inference_interval_s": 0.1,
        "source_frame_age_limit_s": 0.5, "result_ttl_s": 0.5,
    }


class StreamConfigTest(unittest.TestCase):
    def test_full_config_roundtrip_and_summary(self):
        text = json.dumps(config_values())
        config = loads_config(text)
        self.assertIsInstance(config, StreamConfig)
        self.assertEqual(config.video_port, 5000)
        self.assertEqual(summary(config)["role"], "sender")

    def test_missing_and_unknown_keys_rejected(self):
        missing = config_values()
        del missing["video_idle_timeout_s"]
        with self.assertRaises(ProtocolError):
            from_mapping(missing)
        unknown = {**config_values(), "password": "should-not-be-configured"}
        with self.assertRaises(ProtocolError):
            from_mapping(unknown)

    def test_bool_integer_nonfinite_and_ranges_rejected(self):
        for field, value in (("video_port", True), ("max_width", 1921),
                             ("session_lease_s", float("nan")),
                             ("session_lease_s", 10 ** 1000),
                             ("heartbeat_interval_s", 0.6)):
            with self.subTest(field=field):
                with self.assertRaises(ProtocolError):
                    from_mapping({**config_values(), field: value})
        with self.assertRaises(ProtocolError):
            loads_config(json.dumps(config_values()).replace('"video_port": 5000', '"video_port": NaN'))

    def test_duplicate_json_key_rejected(self):
        text = json.dumps(config_values())[:-1] + ',"video_port":5000}'
        with self.assertRaises(ProtocolError):
            loads_config(text)

    def test_bad_host_and_equal_ports_rejected(self):
        with self.assertRaises(ProtocolError):
            from_mapping({**config_values(), "service_host": "bad host"})
        with self.assertRaises(ProtocolError):
            from_mapping({**config_values(), "result_port": 5000})


if __name__ == "__main__":
    unittest.main()

