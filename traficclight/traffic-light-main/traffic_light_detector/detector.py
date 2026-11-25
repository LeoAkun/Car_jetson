"""Stateless RED/GREEN/UNKNOWN observations with explicit mock provenance."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import re
import time
from typing import Optional

import cv2
import numpy as np

from .contract import OUTPUT_SHAPE


@dataclass(frozen=True)
class FrameContext:
    source_camera_id: str
    source_boot_id: str
    source_frame_seq: int
    source_capture_monotonic_ns: int
    source_mode: str = "mock"

    def valid_mock(self):
        return (self.source_mode == "mock"
                and all(isinstance(v, str) and 0 < len(v) <= 128 and v.isprintable()
                        for v in (self.source_camera_id, self.source_boot_id))
                and type(self.source_frame_seq) is int and 0 <= self.source_frame_seq < 2**64
                and type(self.source_capture_monotonic_ns) is int
                and 0 < self.source_capture_monotonic_ns < 2**64)


@dataclass(frozen=True)
class DetectorConfig:
    # Development probes only. Neither threshold has been calibrated on real lamps.
    confidence_threshold: float = 0.5
    max_bbox_area_fraction: float = 0.08

    def __post_init__(self):
        for value in (self.confidence_threshold, self.max_bbox_area_fraction):
            if isinstance(value, bool) or not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError("thresholds must be finite and in (0, 1]")


@dataclass(frozen=True)
class Candidate:
    output_index: int
    class_id: float
    color: str
    confidence: float
    bbox_input: tuple
    bbox: Optional[tuple]
    bbox_normalized: Optional[tuple]
    rejection: Optional[str]


@dataclass(frozen=True)
class Observation:
    state: str
    valid: bool
    confidence: float
    selected: Optional[Candidate]
    candidates: tuple
    reason: str
    source_context: Optional[FrameContext]
    received_monotonic_ns: int
    inference_ms: float
    model_sha256: str
    model_version: str
    config: DetectorConfig
    roi: Optional[tuple]

    @property
    def has_bbox(self):
        return self.selected is not None

    def to_dict(self):
        payload = asdict(self)
        context = payload.pop("source_context")
        payload.update(context or {name: None for name in FrameContext.__dataclass_fields__})
        payload.update(has_bbox=self.has_bbox, control_eligible=False,
                       automatic_release_enabled=False, thresholds_calibrated=False)
        return payload


@dataclass(frozen=True)
class Transform:
    image_width: int
    image_height: int
    crop: tuple
    scaled_width: int
    scaled_height: int
    left: int
    top: int

    def restore(self, bbox):
        x0, y0, x1, y1 = self.crop
        # Actual rounded resize scales avoid a subpixel error on odd dimensions.
        sx, sy = self.scaled_width / (x1 - x0), self.scaled_height / (y1 - y0)
        x_min, y_min, x_max, y_max = bbox
        return ((x_min - self.left) / sx + x0, (y_min - self.top) / sy + y0,
                (x_max - self.left) / sx + x0, (y_max - self.top) / sy + y0)


def prepare(frame, roi):
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3
            or frame.shape[2] != 3 or min(frame.shape[:2]) < 1):
        raise ValueError("invalid_frame")
    if (not isinstance(roi, (tuple, list)) or len(roi) != 4
            or any(isinstance(v, (bool, str)) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in roi)):
        raise ValueError("invalid_roi")
    rx0, ry0, rx1, ry1 = roi
    if not (0 <= rx0 < rx1 <= 1 and 0 <= ry0 < ry1 <= 1):
        raise ValueError("invalid_roi")
    height, width = frame.shape[:2]
    # Crop inside the requested normalized ROI; never add pixels from outside it.
    x0, y0 = math.ceil(rx0 * width), math.ceil(ry0 * height)
    x1, y1 = math.floor(rx1 * width), math.floor(ry1 * height)
    if x1 <= x0 or y1 <= y0:
        raise ValueError("empty_roi")
    crop = frame[y0:y1, x0:x1]
    ratio = min(640 / crop.shape[0], 640 / crop.shape[1])
    scaled_width = max(1, round(crop.shape[1] * ratio))
    scaled_height = max(1, round(crop.shape[0] * ratio))
    left, top = (640 - scaled_width) // 2, (640 - scaled_height) // 2
    pixels = np.full((640, 640, 3), 114, np.uint8)
    pixels[top:top + scaled_height, left:left + scaled_width] = cv2.resize(
        crop, (scaled_width, scaled_height), interpolation=cv2.INTER_LINEAR)
    pixels = np.ascontiguousarray(pixels[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)
    pixels /= np.float32(255)
    return pixels, Transform(width, height, (x0, y0, x1, y1), scaled_width, scaled_height, left, top)


def decode(output, transform, config):
    if (not isinstance(output, np.ndarray) or output.shape != OUTPUT_SHAPE
            or output.dtype != np.float32 or not np.isfinite(output).all()):
        return (), None, "invalid_output_tensor"
    if np.any(output[0, :, 4] < 0) or np.any(output[0, :, 4] > 1):
        return (), None, "invalid_confidence"
    candidates = []
    malformed = False
    for index, row in enumerate(output[0]):
        x0, y0, x1, y1, confidence, class_id = map(float, row)
        color = {0.0: "RED", 1.0: "GREEN"}.get(class_id, "UNKNOWN")
        bbox, normalized, rejection = None, None, None
        if confidence < config.confidence_threshold:
            rejection = "low_confidence"
        elif color == "UNKNOWN":
            rejection, malformed = "unsupported_class", True
        elif not (0 <= x0 < x1 <= 640 and 0 <= y0 < y1 <= 640):
            rejection, malformed = "invalid_bbox", True
        else:
            bbox = transform.restore((x0, y0, x1, y1))
            normalized = tuple(value / scale for value, scale in zip(
                bbox, (transform.image_width, transform.image_height) * 2))
            cx0, cy0, cx1, cy1 = transform.crop
            if not (cx0 <= bbox[0] < bbox[2] <= cx1 and cy0 <= bbox[1] < bbox[3] <= cy1):
                rejection = "outside_roi_or_padding"
            elif ((bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                  / (transform.image_width * transform.image_height) > config.max_bbox_area_fraction):
                rejection = "area_too_large"
        candidates.append(Candidate(index, class_id, color, confidence,
                                    (x0, y0, x1, y1), bbox, normalized, rejection))
    candidates = tuple(candidates)
    if malformed:
        return candidates, None, "malformed_candidate"
    eligible = [candidate for candidate in candidates if candidate.rejection is None]
    if not eligible:
        reason = "no_detection" if not np.any(output[0, :, 4]) else "no_eligible_candidate"
        return candidates, None, reason
    if len({candidate.color for candidate in eligible}) > 1:
        return candidates, None, "conflicting_colors"
    # Several spatial targets cannot be attributed to the current movement in P4.
    # The graph is already one2one/TopK, so do not silently NMS or choose by confidence.
    if len(eligible) > 1:
        return candidates, None, "ambiguous_targets"
    return candidates, eligible[0], "detected"


class Detector:
    def __init__(self, runner, config=None):
        if not re.fullmatch(r"[0-9a-f]{64}", runner.model_sha256):
            raise ValueError("runner must provide a pinned model SHA-256")
        self.runner = runner
        self.config = config or DetectorConfig()

    def detect(self, frame, roi, *, context=None, received_monotonic_ns=None):
        received = time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns
        candidates, selected, inference_ms, canonical_roi = (), None, 0.0, None
        safe_context = context if isinstance(context, FrameContext) and context.valid_mock() else None

        def observation(reason):
            return Observation(selected.color if selected else "UNKNOWN", selected is not None,
                               selected.confidence if selected else 0.0, selected, candidates, reason,
                               safe_context, received, inference_ms, self.runner.model_sha256,
                               self.runner.model_version, self.config, canonical_roi)

        if type(received) is not int or received <= 0 or received > time.monotonic_ns():
            received = time.monotonic_ns()
            return observation("invalid_receive_time")
        if safe_context is None:
            return observation("missing_or_unverified_source_context")
        try:
            pixels, transform = prepare(frame, roi)
            canonical_roi = tuple(float(value) for value in roi)
        except ValueError as error:
            return observation(str(error))
        except Exception:
            return observation("preprocess_error")
        start = time.perf_counter_ns()
        try:
            output = self.runner.infer(pixels)
            inference_ms = (time.perf_counter_ns() - start) / 1e6
            candidates, selected, reason = decode(output, transform, self.config)
            return observation(reason)
        except Exception:
            inference_ms = (time.perf_counter_ns() - start) / 1e6
            selected = None
            return observation("inference_or_decode_error")
