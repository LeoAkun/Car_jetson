"""Validate the candidate manifest and engine before initializing CUDA."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re


CLASSES = {"0": "red", "1": "green"}
INPUT_SHAPE = (1, 3, 640, 640)
OUTPUT_SHAPE = (1, 300, 6)


def load_candidate(engine_path: Path, manifest_path: Path, expected_sha256: str):
    """The caller pins the hash independently; a manifest alone is not a trust root."""
    if not isinstance(expected_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_sha256):
        raise ValueError("expected engine SHA-256 is required")
    manifest = json.loads(Path(manifest_path).read_text())
    required = {
        "engine_sha256": expected_sha256,
        "classes": CLASSES,
        "input_name": "images", "input_shape": list(INPUT_SHAPE), "input_dtype": "float32",
        "output_name": "output0", "output_shape": list(OUTPUT_SHAPE), "output_dtype": "float32",
        "output_columns": ["x1", "y1", "x2", "y2", "confidence", "class_id"],
        "output_coordinates": "pixels in the 640x640 letterboxed input",
        "engine_container": "ultralytics_length_prefixed_json",
        "automatic_release_enabled": False,
        "production_confidence_threshold": None,
    }
    for name, value in required.items():
        if name not in manifest or manifest[name] != value:
            raise ValueError(f"manifest mismatch: {name}")
    if manifest.get("status") != "candidate_requires_labeled_validation":
        raise ValueError("this observer accepts the uncalibrated P3 candidate only")
    if not isinstance(manifest.get("model_version"), str) or not manifest["model_version"]:
        raise ValueError("model_version is required")
    preprocess = manifest.get("preprocess", {})
    for name, value in {"color": "RGB", "layout": "NCHW", "normalization": "uint8 / 255.0",
                        "resize": "aspect-preserving centered letterbox", "padding_uint8": 114,
                        "roi": "caller selects ROI before letterbox and retains coordinate transform"}.items():
        if preprocess.get(name) != value:
            raise ValueError(f"preprocess mismatch: {name}")
    container = Path(engine_path).read_bytes()
    if hashlib.sha256(container).hexdigest() != expected_sha256:
        raise ValueError("engine SHA-256 mismatch")
    header_size = int.from_bytes(container[:4], "little")
    if not 1 <= header_size <= 65536 or 4 + header_size >= len(container):
        raise ValueError("invalid engine metadata length")
    metadata = json.loads(container[4:4 + header_size])
    for name, value in {"names": CLASSES, "source_sha256": manifest.get("source_sha256"),
                        "batch": 1, "imgsz": [640, 640], "precision": "fp16",
                        "task": "detect", "tensorrt": manifest.get("tensorrt"),
                        "output_format": "xyxy_pixel,confidence,class_id; top-300 one2one; no NMS"}.items():
        if metadata.get(name) != value:
            raise ValueError(f"engine metadata mismatch: {name}")
    for name in ("source_sha256", "onnx_sha256"):
        if not isinstance(manifest.get(name), str) or not re.fullmatch(r"[0-9a-f]{64}", manifest[name]):
            raise ValueError(f"invalid {name}")
    return manifest, metadata, container[4 + header_size:]


def validate_io(tensors):
    """Check the actual deserialized engine, independently of the JSON declaration."""
    expected = [("images", "INPUT", INPUT_SHAPE, "float32"),
                ("output0", "OUTPUT", OUTPUT_SHAPE, "float32")]
    actual = [(name, mode, tuple(shape), dtype) for name, mode, shape, dtype in tensors]
    if sorted(actual) != sorted(expected):
        raise ValueError(f"engine I/O contract mismatch: {actual}")
