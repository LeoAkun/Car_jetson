import importlib.util
import hashlib
import json
import socket
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "self_check.py"
SPEC = importlib.util.spec_from_file_location("traffic_light_self_check", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class SelfCheckTest(unittest.TestCase):
    def test_optional_failure_is_warning(self):
        outcome = MODULE.result("manifest", False, "missing", required=False)
        self.assertEqual(outcome.status, "WARN")
        self.assertFalse(outcome.required)

    def test_required_failure_is_failure(self):
        outcome = MODULE.result("cuda", False, "not available")
        self.assertEqual(outcome.status, "FAIL")
        self.assertTrue(outcome.required)

    def test_port_check_detects_an_occupied_port(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        try:
            port = listener.getsockname()[1]
            outcome = MODULE.check_ports("127.0.0.1", [port])
        finally:
            listener.close()
        self.assertEqual(outcome.status, "FAIL")
        self.assertIn(str(port), outcome.detail)

    def test_port_check_accepts_a_free_port(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()

        outcome = MODULE.check_ports("127.0.0.1", [port])
        self.assertEqual(outcome.status, "PASS")

    def test_raw_engine_is_passed_to_tensorrt_unchanged(self):
        payload = b"ptrt" + bytes(32)
        self.assert_engine_input(payload, payload, 0)

    def test_ultralytics_wrapper_is_removed_before_deserialization(self):
        payload = b"ptrt" + bytes(32)
        metadata = json.dumps({"names": {"0": "red", "1": "green"}}).encode()
        wrapped = len(metadata).to_bytes(4, "little") + metadata + payload
        self.assert_engine_input(wrapped, payload, 4 + len(metadata))

    def test_invalid_metadata_is_not_silently_stripped(self):
        payload = (5).to_bytes(4, "little") + b"abcde" + b"ptrt" + bytes(32)
        self.assert_engine_input(payload, payload, 0)

    def test_candidate_engine_uses_p4_contract_and_independent_hash(self):
        payload = b"ptrt" + bytes(32)
        manifest = {
            "model_version": "fixture-candidate", "status": "candidate_requires_labeled_validation",
            "classes": {"0": "red", "1": "green"},
            "input_name": "images", "input_shape": [1, 3, 640, 640], "input_dtype": "float32",
            "output_name": "output0", "output_shape": [1, 300, 6], "output_dtype": "float32",
            "output_columns": ["x1", "y1", "x2", "y2", "confidence", "class_id"],
            "output_coordinates": "pixels in the 640x640 letterboxed input",
            "engine_container": "ultralytics_length_prefixed_json",
            "automatic_release_enabled": False, "production_confidence_threshold": None,
            "source_sha256": "a" * 64, "onnx_sha256": "b" * 64, "tensorrt": "10.7.0",
            "preprocess": {"color": "RGB", "layout": "NCHW", "normalization": "uint8 / 255.0",
                           "resize": "aspect-preserving centered letterbox", "padding_uint8": 114,
                           "roi": "caller selects ROI before letterbox and retains coordinate transform"},
        }
        metadata = {
            "names": manifest["classes"], "source_sha256": manifest["source_sha256"],
            "batch": 1, "imgsz": [640, 640], "precision": "fp16", "task": "detect",
            "tensorrt": manifest["tensorrt"],
            "output_format": "xyxy_pixel,confidence,class_id; top-300 one2one; no NMS",
        }
        header = json.dumps(metadata).encode()
        wrapped = len(header).to_bytes(4, "little") + header + payload
        engine_hash = hashlib.sha256(wrapped).hexdigest()
        manifest["engine_sha256"] = engine_hash

        trt = MagicMock()
        trt.__version__ = "10.7.0"
        runtime = trt.Runtime.return_value
        trt.nptype.return_value = np.float32
        deserialized = runtime.deserialize_cuda_engine.return_value
        deserialized.num_io_tensors = 2
        deserialized.get_tensor_name.side_effect = ["images", "output0"]
        deserialized.get_tensor_mode.side_effect = lambda name: "INPUT" if name == "images" else "OUTPUT"
        deserialized.get_tensor_dtype.return_value = "float32"
        deserialized.get_tensor_shape.side_effect = lambda name: (
            (1, 3, 640, 640) if name == "images" else (1, 300, 6)
        )
        with tempfile.TemporaryDirectory() as directory:
            engine = Path(directory) / "sample.engine"
            manifest_path = Path(directory) / "model_manifest.json"
            engine.write_bytes(wrapped)
            manifest_path.write_text(json.dumps(manifest))
            with patch.object(MODULE, "import_module", return_value=(trt, None)):
                outcome = MODULE.check_tensorrt([engine], manifest_path, engine_hash)
        self.assertEqual(outcome.status, "PASS")
        self.assertEqual(outcome.data["engines"][0]["candidate_contract"], "validated")
        self.assertEqual(outcome.data["engines"][0]["io_contract"], "validated")
        runtime.deserialize_cuda_engine.assert_called_once_with(payload)

    def test_candidate_hash_mismatch_fails_before_deserialization(self):
        trt = MagicMock()
        trt.__version__ = "10.7.0"
        with tempfile.TemporaryDirectory() as directory:
            engine = Path(directory) / "sample.engine"
            manifest = Path(directory) / "model_manifest.json"
            engine.write_bytes(b"not-a-valid-container")
            manifest.write_text("{}")
            with patch.object(MODULE, "import_module", return_value=(trt, None)) as import_module:
                outcome = MODULE.check_tensorrt([engine], manifest, "0" * 64)
        self.assertEqual(outcome.status, "FAIL")
        import_module.assert_not_called()
        trt.Runtime.return_value.deserialize_cuda_engine.assert_not_called()

    def assert_engine_input(self, file_bytes, expected_payload, header_bytes):
        trt = MagicMock()
        trt.__version__ = "10.7.0"
        runtime = trt.Runtime.return_value
        runtime.deserialize_cuda_engine.return_value.num_io_tensors = 0
        with tempfile.TemporaryDirectory() as directory:
            engine = Path(directory) / "sample.engine"
            engine.write_bytes(file_bytes)
            with patch.object(MODULE, "import_module", return_value=(trt, None)):
                outcome = MODULE.check_tensorrt([engine])
        self.assertEqual(outcome.status, "PASS")
        self.assertEqual(outcome.data["engines"][0]["metadata_header_bytes"], header_bytes)
        runtime.deserialize_cuda_engine.assert_called_once_with(expected_payload)


if __name__ == "__main__":
    unittest.main()
