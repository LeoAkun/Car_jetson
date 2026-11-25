import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from traffic_light_detector.contract import load_candidate, validate_io
from traffic_light_detector.detector import Detector, DetectorConfig, FrameContext, prepare


def output(*rows):
    tensor = np.zeros((1, 300, 6), dtype=np.float32)
    if rows:
        tensor[0, :len(rows)] = rows
    return tensor


GREEN = [100, 100, 120, 140, 0.9, 1]
RED = [200, 100, 220, 140, 0.9, 0]


class Runner:
    model_sha256 = "a" * 64
    model_version = "unit-test-model"

    def __init__(self, result):
        self.result = result
        self.calls = 0

    def infer(self, pixels):
        self.calls += 1
        self.last_pixels = pixels.copy()
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


class DetectorTest(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((640, 640, 3), np.uint8)
        self.context = FrameContext("mock-front", "mock-epoch-1", 4, 123456789)
        self.runner = Runner(output(GREEN))
        self.detector = Detector(self.runner)

    def detect(self, frame=None, roi=(0, 0, 1, 1), **kwargs):
        return self.detector.detect(self.frame if frame is None else frame, roi,
                                    context=kwargs.pop("context", self.context), **kwargs)

    def test_no_detection(self):
        self.runner.result = output()
        result = self.detect()
        self.assertEqual((result.state, result.reason, result.has_bbox), ("UNKNOWN", "no_detection", False))

    def test_green_red_empty_and_failure_never_latch(self):
        for tensor, state in [(output(GREEN), "GREEN"), (output(RED), "RED"),
                              (output(GREEN), "GREEN"), (output(), "UNKNOWN"),
                              (output(GREEN), "GREEN"), (RuntimeError("GPU"), "UNKNOWN")]:
            self.runner.result = tensor
            self.assertEqual(self.detect().state, state)

    def test_threshold_inclusive_and_below(self):
        row = GREEN.copy()
        for confidence, state in [(0.5, "GREEN"), (np.nextafter(np.float32(0.5), np.float32(0)), "UNKNOWN")]:
            row[4] = confidence
            self.runner.result = output(row)
            self.assertEqual(self.detect().state, state)

    def test_multiple_same_color_is_ambiguous(self):
        self.runner.result = output(GREEN, [200, 100, 220, 140, .99, 1])
        result = self.detect()
        self.assertEqual((result.state, result.reason), ("UNKNOWN", "ambiguous_targets"))
        self.assertEqual(sum(c.rejection is None for c in result.candidates), 2)

    def test_conflict_ignores_confidence_order(self):
        for rows in [(GREEN, RED), (RED, GREEN)]:
            self.runner.result = output(*rows)
            self.assertEqual(self.detect().reason, "conflicting_colors")

    def test_unsupported_or_fractional_class_poisons_green(self):
        for class_id in (2, -1, 0.5):
            row = RED.copy()
            row[5] = class_id
            self.runner.result = output(GREEN, row)
            self.assertEqual((self.detect().state, self.detect().reason), ("UNKNOWN", "malformed_candidate"))

    def test_tensor_contract_and_nonfinite(self):
        for bad in [None, np.zeros((300, 6), np.float32), output().astype(np.float16),
                    np.full((1, 300, 6), np.nan, np.float32),
                    np.full((1, 300, 6), np.inf, np.float32)]:
            with self.subTest(value=type(bad)):
                self.runner.result = bad
                self.assertEqual(self.detect().reason, "invalid_output_tensor")

    def test_invalid_confidence_anywhere_poisons_frame(self):
        for value in (-.01, 1.01):
            tensor = output(GREEN)
            tensor[0, -1, 4] = value
            self.runner.result = tensor
            self.assertEqual(self.detect().reason, "invalid_confidence")

    def test_invalid_positive_coordinates_poisons_green(self):
        for coords in [(-1, 0, 20, 30), (2, 0, 1, 30), (0, 0, 641, 30), (0, 2, 20, 2)]:
            self.runner.result = output(GREEN, [*coords, .9, 0])
            self.assertEqual(self.detect().state, "UNKNOWN")

    def test_area_filter_uses_original_image_fraction(self):
        self.runner.result = output([0, 0, 640, 640, .99, 1])
        self.assertEqual(self.detect().candidates[0].rejection, "area_too_large")

    def test_boundary_normalization_zero_one(self):
        self.runner.result = output([0, 0, 640, 10, .9, 1])
        result = self.detect()
        self.assertEqual(result.state, "GREEN")
        self.assertEqual(result.selected.bbox_normalized, (0, 0, 1, 10 / 640))

    def test_invalid_frame_and_roi_skip_inference(self):
        frames = [np.zeros((0, 2, 3), np.uint8), np.zeros((2, 2), np.uint8),
                  np.zeros((2, 2, 4), np.uint8), self.frame.astype(float), "bad"]
        for frame in frames:
            self.assertEqual(self.detect(frame=frame).reason, "invalid_frame")
        for roi in [(0, 0, 0, 1), (-.1, 0, 1, 1), (0, 0, 1.1, 1),
                    (0, 0, float("nan"), 1), (False, 0, 1, 1), (0, 1), None]:
            self.assertEqual(self.detect(roi=roi).reason, "invalid_roi")
        self.assertEqual(self.detect(roi=(0, 0, 0.00001, 0.00001)).reason, "empty_roi")
        self.assertEqual(self.runner.calls, 0)

    def test_rgb_padding_and_crop_excludes_outside_pixels(self):
        frame = np.zeros((320, 640, 3), np.uint8)
        frame[:, :160] = [0, 255, 0]  # green outside the requested ROI
        frame[:, 160:480] = [0, 0, 255]
        self.runner.result = output()
        self.detect(frame=frame, roi=(.25, 0, .75, 1))
        np.testing.assert_array_equal(self.runner.last_pixels[0, :, 320, 320], [1, 0, 0])
        self.assertFalse(np.any(self.runner.last_pixels[0, 1]))
        pixels, transform = prepare(frame, (0, 0, 1, 1))
        self.assertEqual((pixels.shape, pixels.dtype, pixels.flags.c_contiguous), ((1, 3, 640, 640), np.float32, True))
        np.testing.assert_allclose(pixels[0, :, 0, 0], np.full(3, 114 / 255), atol=1e-7)
        self.assertEqual(transform.top, 160)

    def test_outside_roi_padding_never_green(self):
        # Crop is 320x640 and occupies x=160..480 in letterbox coordinates.
        self.runner.result = output([20, 100, 40, 140, .99, 1])
        result = self.detect(roi=(.25, 0, .75, 1))
        self.assertEqual(result.state, "UNKNOWN")
        self.assertEqual(result.candidates[0].rejection, "outside_roi_or_padding")

    def test_letterbox_roi_restore_on_odd_sizes(self):
        frame = np.zeros((431, 717, 3), np.uint8)
        roi = (.13, .07, .89, .86)
        _, transform = prepare(frame, roi)
        cx0, cy0, cx1, cy1 = transform.crop
        original = np.array([cx0 + 7, cy0 + 13, cx0 + 22, cy0 + 48], dtype=float)
        scales = np.array([transform.scaled_width / (cx1 - cx0), transform.scaled_height / (cy1 - cy0)] * 2)
        translated = np.array([cx0, cy0, cx0, cy0])
        padding = np.array([transform.left, transform.top] * 2)
        self.runner.result = output([*(scales * (original - translated) + padding), .99, 0])
        result = self.detect(frame=frame, roi=roi)
        self.assertEqual(result.state, "RED")
        np.testing.assert_allclose(result.selected.bbox, original, atol=1e-4)
        np.testing.assert_allclose(result.selected.bbox_normalized,
                                   original / [717, 431, 717, 431], atol=1e-6)

    def test_mock_context_passthrough_and_control_always_disabled(self):
        received = time.monotonic_ns()
        result = self.detect(received_monotonic_ns=received)
        payload = result.to_dict()
        for name, value in self.context.__dict__.items():
            self.assertEqual(payload[name], value)
        self.assertEqual(payload["received_monotonic_ns"], received)
        self.assertGreaterEqual(payload["inference_ms"], 0)
        self.assertTrue(payload["valid"])
        self.assertFalse(payload["control_eligible"])
        self.assertFalse(payload["automatic_release_enabled"])
        self.assertFalse(payload["thresholds_calibrated"])
        json.dumps(payload, allow_nan=False)

    def test_missing_real_or_malformed_context_cannot_green(self):
        bad_contexts = [None, {}, FrameContext("front", "epoch", 1, 2, "real"),
                        FrameContext("", "epoch", 1, 2), FrameContext("front", "epoch", True, 2),
                        FrameContext("front", "epoch", -1, 2), FrameContext("front", "epoch", 1, 0)]
        for context in bad_contexts:
            self.assertEqual(self.detect(context=context).state, "UNKNOWN")
        self.assertEqual(self.runner.calls, 0)

    def test_invalid_receive_time(self):
        for received in (True, -1, time.monotonic_ns() + 10**15):
            self.assertEqual(self.detect(received_monotonic_ns=received).reason, "invalid_receive_time")

    def test_context_and_pixels_not_reused_between_calls(self):
        first = self.detect()
        next_context = FrameContext("other-camera", "new-epoch", 0, 999)
        self.runner.result = output(RED)
        second = self.detect(context=next_context)
        self.assertEqual((first.state, second.state), ("GREEN", "RED"))
        self.assertEqual(first.source_context, self.context)
        self.assertEqual(second.source_context, next_context)

    def test_repeatability_and_candidate_audit_is_not_mutable(self):
        first, second = self.detect(), self.detect()
        self.assertEqual(first.candidates, second.candidates)
        self.assertEqual(len(first.candidates), 300)
        self.assertEqual(first.candidates[1].rejection, "low_confidence")
        self.runner.result.fill(0)
        self.assertEqual(first.selected.color, "GREEN")

    def test_bad_config(self):
        for value in (0, -1, float("nan"), float("inf"), 1.1, True):
            with self.assertRaises(ValueError):
                DetectorConfig(confidence_threshold=value)


class ContractTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.manifest = {
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
        # Self-contained fixture metadata; never deserialize a GPU plan in CPU tests.
        self.metadata = {"names": {"0": "red", "1": "green"}, "source_sha256": self.manifest["source_sha256"],
                         "batch": 1, "imgsz": [640, 640], "precision": "fp16", "task": "detect",
                         "tensorrt": self.manifest["tensorrt"],
                         "output_format": "xyxy_pixel,confidence,class_id; top-300 one2one; no NMS"}
        self.write_fixture()

    def write_fixture(self):
        header = json.dumps(self.metadata).encode()
        self.engine = self.root / "test.engine"
        self.engine.write_bytes(len(header).to_bytes(4, "little") + header + b"unit-test-plan")
        self.sha = hashlib.sha256(self.engine.read_bytes()).hexdigest()
        self.manifest["engine_sha256"] = self.sha
        self.path = self.root / "model_manifest.json"
        self.path.write_text(json.dumps(self.manifest))

    def load(self):
        return load_candidate(self.engine, self.path, self.sha)

    def test_valid_container(self):
        manifest, metadata, plan = self.load()
        self.assertEqual(plan, b"unit-test-plan")
        self.assertEqual(metadata["names"], manifest["classes"])

    def test_engine_tamper_rejected_before_cuda(self):
        self.engine.write_bytes(self.engine.read_bytes() + b"tampered")
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            self.load()

    def test_manifest_mismatches_rejected(self):
        original = copy.deepcopy(self.manifest)
        for name, wrong in [("classes", {"0": "green", "1": "red"}), ("input_shape", [1, 3, 320, 320]),
                            ("input_dtype", "float16"), ("output_shape", [1, 6, 300]),
                            ("output_dtype", "float16"), ("engine_sha256", "f" * 64),
                            ("automatic_release_enabled", True), ("production_confidence_threshold", .5)]:
            with self.subTest(field=name):
                self.manifest = {**original, name: wrong}
                self.path.write_text(json.dumps(self.manifest))
                with self.assertRaises(ValueError):
                    self.load()

    def test_preprocess_contract_mismatch(self):
        self.manifest["preprocess"]["color"] = "BGR"
        self.path.write_text(json.dumps(self.manifest))
        with self.assertRaisesRegex(ValueError, "preprocess"):
            self.load()

    def test_engine_metadata_mismatch(self):
        for name, value in [("names", {"0": "go"}), ("source_sha256", "b" * 64),
                            ("imgsz", [320, 320]), ("output_format", "raw-yolo"), ("tensorrt", "8.0")]:
            with self.subTest(field=name):
                original = copy.deepcopy(self.metadata)
                self.metadata[name] = value
                self.write_fixture()
                with self.assertRaisesRegex(ValueError, "metadata"):
                    self.load()
                self.metadata = original

    def test_bad_container_lengths(self):
        for container in (b"", b"ptrt" + b"payload", (100).to_bytes(4, "little") + b"{}"):
            self.engine.write_bytes(container)
            self.sha = hashlib.sha256(container).hexdigest()
            self.manifest["engine_sha256"] = self.sha
            self.path.write_text(json.dumps(self.manifest))
            with self.assertRaises(ValueError):
                self.load()

    def test_actual_engine_io_mismatch(self):
        good = [("images", "INPUT", (1, 3, 640, 640), "float32"),
                ("output0", "OUTPUT", (1, 300, 6), "float32")]
        validate_io(good[::-1])
        for wrong in [good[:1], good + good[:1],
                      [good[0], ("output0", "OUTPUT", (1, 300, 6), "float16")],
                      [("images", "OUTPUT", (1, 3, 640, 640), "float32"), good[1]]]:
            with self.assertRaises(ValueError):
                validate_io(wrong)


if __name__ == "__main__":
    unittest.main()
