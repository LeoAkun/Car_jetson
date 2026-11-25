import importlib.util
import unittest
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "build_jetson_model.py"
SPEC = importlib.util.spec_from_file_location("traffic_light_build", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ModelOutputContractTest(unittest.TestCase):
    def setUp(self):
        self.reference = np.zeros((1, 300, 6), dtype=np.float32)
        self.reference[0, 0] = [10, 20, 30, 40, 0.9, 0]
        self.reference[0, 1] = [60, 70, 80, 90, 0.8, 1]

    def compare(self, actual):
        return MODULE.compare_outputs(self.reference, actual,
                                      confidence_tolerance=0.03, box_tolerance=3.0)

    def test_topk_reordering_preserves_equivalent_detections(self):
        actual = self.reference.copy()
        actual[0, [0, 1]] = actual[0, [1, 0]]
        self.assertEqual(self.compare(actual)["status"], "PASS")

    def test_red_green_swap_is_rejected_with_identical_boxes_and_scores(self):
        actual = self.reference.copy()
        actual[0, :2, 5] = 1 - actual[0, :2, 5]
        self.assertEqual(self.compare(actual)["status"], "FAIL")

    def test_disappearing_confident_detection_is_rejected(self):
        actual = self.reference.copy()
        actual[0, 1, 4] = 0.49
        self.assertEqual(self.compare(actual)["status"], "FAIL")

    def test_shifted_box_is_rejected(self):
        actual = self.reference.copy()
        actual[0, 0, :4] += 4
        self.assertEqual(self.compare(actual)["status"], "FAIL")

    def test_nonfinite_output_is_rejected(self):
        actual = self.reference.copy()
        actual[0, 299, 0] = np.nan
        with self.assertRaises(ValueError):
            self.compare(actual)

    def test_letterbox_keeps_aspect_and_converts_bgr_to_rgb(self):
        image = np.zeros((320, 640, 3), dtype=np.uint8)
        image[:] = [0, 0, 255]
        pixels = MODULE.letterbox(image)
        self.assertEqual(pixels.shape, (1, 3, 640, 640))
        np.testing.assert_array_equal(pixels[0, :, 320, 320], [1, 0, 0])
        np.testing.assert_allclose(pixels[0, :, 0, 0], np.full(3, 114 / 255), atol=1e-7)


if __name__ == "__main__":
    unittest.main()
