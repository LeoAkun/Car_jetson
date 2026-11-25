"""Offline candidate probe. Does not open a camera, socket, ROS node or service."""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from .detector import Detector, DetectorConfig, FrameContext
from .engine import TensorRTRunner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--image", type=Path, action="append", default=[])
    parser.add_argument("--synthetic-smoke", action="store_true")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--roi", type=float, nargs=4, default=[0, 0, 1, 1])
    parser.add_argument("--probe-confidence", type=float, default=0.5)
    parser.add_argument("--include-candidates", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.repeat <= 1000 or not (args.image or args.synthetic_smoke):
        parser.error("provide images and/or --synthetic-smoke; repeat must be 1..1000")
    try:
        runner = TensorRTRunner(args.engine, args.manifest, args.expected_sha256)
        detector = Detector(runner, DetectorConfig(confidence_threshold=args.probe_confidence))
        images = []
        for path in args.image:
            frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError(f"cannot decode image: {path}")
            images.append((str(path), frame))
        if args.synthetic_smoke:
            images.extend([
                ("synthetic_black", np.zeros((360, 640, 3), np.uint8)),
                ("synthetic_white", np.full((361, 641, 3), 255, np.uint8)),
                ("synthetic_random_seed_42", np.random.default_rng(42).integers(
                    0, 256, (480, 640, 3), dtype=np.uint8)),
            ])
        boot = "offline-probe-" + str(time.monotonic_ns())
        baselines, samples, timings = {}, [], []
        # Warm all TensorRT paths before measuring; every invocation uses fresh mock context.
        for i in range(5):
            warm = detector.detect(images[0][1], args.roi, context=FrameContext(
                "offline-file", boot, i, time.monotonic_ns()))
            if warm.reason not in {"detected", "conflicting_colors", "ambiguous_targets",
                                   "no_detection", "no_eligible_candidate"}:
                raise RuntimeError(f"warmup failed: {warm.reason}")
        for repeat in range(args.repeat):
            for index, (name, frame) in enumerate(images):
                sequence = 5 + repeat * len(images) + index
                result = detector.detect(frame, args.roi, context=FrameContext(
                    "offline-file", boot, sequence, time.monotonic_ns()))
                payload = result.to_dict()
                if result.reason not in {"detected", "conflicting_colors", "ambiguous_targets",
                                         "no_detection", "no_eligible_candidate"}:
                    raise RuntimeError(f"inference failed: {name}: {result.reason}")
                if payload["control_eligible"] or payload["automatic_release_enabled"]:
                    raise RuntimeError("offline observations must never authorize motion")
                fingerprint = hashlib.sha256(json.dumps({key: payload[key] for key in (
                    "state", "reason", "confidence", "selected", "candidates", "roi")},
                    sort_keys=True, allow_nan=False).encode()).hexdigest()
                if index in baselines and baselines[index] != fingerprint:
                    raise RuntimeError(f"non-repeatable observation: {name}")
                baselines[index] = fingerprint
                timings.append(result.inference_ms)
                if repeat == 0:
                    if not args.include_candidates:
                        payload["candidate_count"] = len(payload.pop("candidates"))
                    samples.append({"input": name, "observation": payload, "fingerprint": fingerprint})
        report = {
            "status": "PASS", "scope": "offline candidate software probe; no real-light accuracy claim",
            "recorded_at_device_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "uid": os.getuid(), "model_sha256": runner.model_sha256,
            "model_version": runner.model_version, "inferences": len(timings), "warmups": 5,
            "repeatable": True, "automatic_release_enabled": False,
            "thresholds_calibrated": False, "samples": samples,
            "inference_ms": {"mean": float(np.mean(timings)), "p50": float(np.percentile(timings, 50)),
                             "p95": float(np.percentile(timings, 95)), "p99": float(np.percentile(timings, 99))},
        }
        print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
        return 0
    except Exception as error:
        print(json.dumps({"status": "FAIL", "error": str(error),
                          "automatic_release_enabled": False}), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
