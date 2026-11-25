#!/usr/bin/env python3
"""Export the repository's legacy YOLOv10 weights and build a TensorRT 10 engine.

Uses the existing JetPack packages. Does not install dependencies or modify the
source checkpoint. Stages write independent reports into an explicit output dir.
"""
from __future__ import annotations

import argparse
import ast
import datetime
import hashlib
import importlib
import json
import os
from pathlib import Path
import platform
import sys
import time
import types


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def now_utc() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def legacy_nn(root: Path):
    """Import the original NN implementation without SDK/training integrations."""
    package_root = root.resolve() / "ultralytics"
    parsed = ast.parse((package_root / "__init__.py").read_text())
    version = next(
        ast.literal_eval(node.value)
        for node in parsed.body
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "__version__" for t in node.targets)
    )
    if "ultralytics" in sys.modules:
        raise RuntimeError("run in a fresh process; ultralytics is already imported")
    package = types.ModuleType("ultralytics")
    package.__file__ = str(package_root / "__init__.py")
    package.__path__ = [str(package_root)]
    package.__version__ = version
    sys.modules["ultralytics"] = package
    importlib.import_module("ultralytics.nn.tasks")
    return version


def prepared_model(args):
    import torch

    version = legacy_nn(args.legacy_root)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    checkpoint = torch.load(args.source, map_location="cpu", weights_only=False)
    model = checkpoint.get("ema") or checkpoint["model"]
    names = {str(key): value for key, value in model.names.items()}
    if names != {"0": "red", "1": "green"}:
        raise ValueError(f"unexpected class mapping: {names}")
    model = model.float().eval().cuda()
    for parameter in model.parameters():
        if not bool(torch.isfinite(parameter).all()):
            raise ValueError("non-finite source parameter")
        parameter.requires_grad_(False)
    model.fuse(verbose=False)
    from ultralytics.nn.modules import C2f, Detect, v10Detect

    for module in model.modules():
        if isinstance(module, Detect):
            module.dynamic = False
            module.export = True
            module.format = "onnx"
            if isinstance(module, v10Detect):
                module.max_det = 300
        elif isinstance(module, C2f):
            module.forward = module.forward_split
    return model, names, version


def export_onnx(args):
    import onnx
    import torch

    path = args.output_dir / "2025_7_24.onnx"
    if path.exists():
        raise FileExistsError(path)
    model, names, version = prepared_model(args)
    pixels = torch.zeros((1, 3, 640, 640), device="cuda")
    with torch.inference_mode():
        for _ in range(2):
            output = model(pixels)
        if list(output.shape) != [1, 300, 6] or not bool(torch.isfinite(output).all()):
            raise RuntimeError("unexpected YOLOv10 export output")
        torch.onnx.export(
            model, pixels, str(path), opset_version=13, do_constant_folding=True,
            input_names=["images"], output_names=["output0"], dynamic_axes=None,
        )
    graph = onnx.load(str(path))
    metadata = {
        "description": "YOLOv10n traffic light red/green detection",
        "version": version, "task": "detect", "stride": 32,
        "batch": 1, "imgsz": [640, 640], "names": names,
        "source_sha256": args.expected_sha256,
        "output_format": "xyxy_pixel,confidence,class_id; top-300 one2one; no NMS",
    }
    for key, value in metadata.items():
        entry = graph.metadata_props.add()
        entry.key, entry.value = key, json.dumps(value)
    onnx.checker.check_model(graph, full_check=True)
    onnx.save(graph, str(path))
    report = {
        "status": "PASS", "recorded_at_device_utc": now_utc(),
        "source_sha256": args.expected_sha256, "onnx_sha256": sha256(path),
        "onnx_bytes": path.stat().st_size, "onnx_version": onnx.__version__,
        "torch_version": torch.__version__, "legacy_version": version,
        "input_shape": [1, 3, 640, 640], "output_shape": [1, 300, 6],
        "opset": 13, "metadata": metadata,
        "legacy_files_sha256": {
            str(p.relative_to(args.legacy_root)): sha256(p)
            for p in sorted((args.legacy_root / "ultralytics").rglob("*"))
            if p.is_file() and p.suffix in {".py", ".yaml"}
        },
    }
    write_json(args.output_dir / "export_report.json", report)
    print(json.dumps({k: v for k, v in report.items() if k != "legacy_files_sha256"}), flush=True)


def build_engine(args):
    import numpy as np
    import tensorrt as trt

    path = args.output_dir / "2025_7_24_fp16.engine"
    if path.exists():
        raise FileExistsError(path)
    exported = json.loads((args.output_dir / "export_report.json").read_text())
    onnx_path = args.output_dir / "2025_7_24.onnx"
    if exported["onnx_sha256"] != sha256(onnx_path):
        raise ValueError("ONNX hash differs from export report")
    if trt.__version__.split(".")[0] != "10":
        raise RuntimeError(f"requires TensorRT 10, got {trt.__version__}")
    logger = trt.Logger(trt.Logger.INFO)
    trt.init_libnvinfer_plugins(logger, "")
    builder = trt.Builder(logger)
    builder.max_threads = 4
    network = builder.create_network(0)
    parser = trt.OnnxParser(network, logger)
    if not parser.parse_from_file(str(onnx_path)):
        raise RuntimeError("\n".join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, args.workspace_mib * 1024**2)
    if not builder.platform_has_fast_fp16:
        raise RuntimeError("FP16 is not available on this GPU")
    config.set_flag(trt.BuilderFlag.FP16)
    config.clear_flag(trt.BuilderFlag.TF32)
    start = time.monotonic()
    print("BUILD_STARTED: FP16 enabled, FP32 I/O, workspace limited", flush=True)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT build returned None")
    metadata = dict(exported["metadata"])
    metadata.update({"tensorrt": trt.__version__, "precision": "fp16", "imgsz": [640, 640]})
    header = json.dumps(metadata).encode("utf-8")
    path.write_bytes(len(header).to_bytes(4, "little") + header + bytes(serialized))
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None:
        raise RuntimeError("new engine cannot be deserialized")
    tensors = []
    for index in range(engine.num_io_tensors):
        name = engine.get_tensor_name(index)
        tensors.append({"name": name, "mode": str(engine.get_tensor_mode(name)),
                        "shape": list(engine.get_tensor_shape(name)),
                        "dtype": str(np.dtype(trt.nptype(engine.get_tensor_dtype(name))))})
    report = {
        "status": "PASS", "recorded_at_device_utc": now_utc(),
        "source_sha256": args.expected_sha256, "onnx_sha256": sha256(onnx_path),
        "engine_sha256": sha256(path), "engine_bytes": path.stat().st_size,
        "build_seconds": time.monotonic() - start, "tensorrt": trt.__version__,
        "workspace_mib": args.workspace_mib, "fp16_enabled": True, "tf32_enabled": False,
        "engine_container": "ultralytics_length_prefixed_json", "io_tensors": tensors,
        "build_script_sha256": sha256(Path(__file__)),
        "l4t": Path("/etc/nv_tegra_release").read_text().strip(),
        "platform": platform.platform(),
    }
    write_json(args.output_dir / "build_report.json", report)
    print(json.dumps(report), flush=True)


def letterbox(image):
    import cv2
    import numpy as np

    height, width = image.shape[:2]
    ratio = min(640 / height, 640 / width)
    resized = cv2.resize(image, (round(width * ratio), round(height * ratio)))
    pad_x, pad_y = (640 - resized.shape[1]) / 2, (640 - resized.shape[0]) / 2
    padded = cv2.copyMakeBorder(
        resized, round(pad_y - 0.1), round(pad_y + 0.1),
        round(pad_x - 0.1), round(pad_x + 0.1), cv2.BORDER_CONSTANT, value=(114, 114, 114),
    )
    return np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.0


class EngineRunner:
    def __init__(self, path):
        import numpy as np
        import tensorrt as trt
        import torch

        serialized = path.read_bytes()
        size = int.from_bytes(serialized[:4], "little")
        self.metadata = json.loads(serialized[4 : 4 + size])
        logger = trt.Logger(trt.Logger.ERROR)
        trt.init_libnvinfer_plugins(logger, "")
        self.runtime = trt.Runtime(logger)
        self.engine = self.runtime.deserialize_cuda_engine(serialized[4 + size :])
        if self.engine is None:
            raise RuntimeError("engine deserialize failed")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("engine context creation failed")
        expected = {"images": (trt.TensorIOMode.INPUT, (1, 3, 640, 640)),
                    "output0": (trt.TensorIOMode.OUTPUT, (1, 300, 6))}
        if self.engine.num_io_tensors != 2:
            raise RuntimeError("unexpected engine tensor count")
        self.buffers = {}
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            mode_shape = (self.engine.get_tensor_mode(name), tuple(self.engine.get_tensor_shape(name)))
            if name not in expected or mode_shape != expected[name]:
                raise RuntimeError(f"unexpected tensor {name}: {mode_shape}")
            dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            if dtype != np.float32:
                raise RuntimeError(f"unexpected I/O dtype: {dtype}")
            self.buffers[name] = torch.empty(expected[name][1], device="cuda", dtype=torch.float32)
            if not self.context.set_tensor_address(name, self.buffers[name].data_ptr()):
                raise RuntimeError(f"cannot bind {name}")
        self.stream = torch.cuda.current_stream()

    def __call__(self, pixels):
        import torch

        start_wall = time.perf_counter()
        self.buffers["images"].copy_(torch.from_numpy(pixels))
        start, end = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
        start.record(self.stream)
        if not self.context.execute_async_v3(self.stream.cuda_stream):
            raise RuntimeError("TensorRT execute_async_v3 failed")
        end.record(self.stream)
        end.synchronize()
        output = self.buffers["output0"].cpu().numpy().copy()
        return output, start.elapsed_time(end), (time.perf_counter() - start_wall) * 1000


def check_output(value):
    import numpy as np

    if value.shape != (1, 300, 6) or not np.isfinite(value).all():
        raise ValueError("non-finite or malformed model output")
    if not ((value[..., 4] >= 0) & (value[..., 4] <= 1)).all():
        raise ValueError("invalid confidence")
    if not np.isin(value[..., 5], [0, 1]).all():
        raise ValueError("invalid class ID")
    if not ((value[..., 2] >= value[..., 0]) & (value[..., 3] >= value[..., 1])).all():
        raise ValueError("invalid xyxy coordinates")


def compare_outputs(reference, actual, *, confidence_tolerance, box_tolerance):
    """Compare score distributions and match confident detections across TopK order."""
    import numpy as np
    from scipy.optimize import linear_sum_assignment

    check_output(reference)
    check_output(actual)
    score_error = float(np.max(np.abs(np.sort(reference[0, :, 4]) - np.sort(actual[0, :, 4]))))
    ref = reference[0][reference[0, :, 4] >= 0.5]
    out = actual[0][actual[0, :, 4] >= 0.5]
    passed = score_error <= confidence_tolerance and len(ref) == len(out)
    report = {"status": "PASS" if passed else "FAIL", "confidence_threshold": 0.5,
              "max_sorted_score_absolute_error": score_error,
              "reference_detections": len(ref), "actual_detections": len(out),
              "confidence_tolerance": confidence_tolerance, "box_pixel_tolerance": box_tolerance,
              "matched_max_box_pixel_error": None, "matched_max_confidence_error": None,
              "class_ids_match": None}
    if len(ref) and len(ref) == len(out):
        box_error = np.max(np.abs(ref[:, None, :4] - out[None, :, :4]), axis=2)
        class_error = ref[:, None, 5] != out[None, :, 5]
        rows, columns = linear_sum_assignment(box_error + class_error * 1e6)
        max_box = float(box_error[rows, columns].max())
        max_score = float(np.abs(ref[rows, 4] - out[columns, 4]).max())
        same_class = bool((ref[rows, 5] == out[columns, 5]).all())
        passed = passed and max_box <= box_tolerance and max_score <= confidence_tolerance and same_class
        report.update({"matched_max_box_pixel_error": max_box,
                       "matched_max_confidence_error": max_score, "class_ids_match": same_class})
    report["status"] = "PASS" if passed else "FAIL"
    return report


def verify_model(args):
    import cv2
    import numpy as np
    import onnxruntime as ort
    import tensorrt as trt
    import torch

    exported = json.loads((args.output_dir / "export_report.json").read_text())
    built = json.loads((args.output_dir / "build_report.json").read_text())
    onnx_path, engine_path = args.output_dir / "2025_7_24.onnx", args.output_dir / "2025_7_24_fp16.engine"
    if sha256(onnx_path) != exported["onnx_sha256"] or sha256(engine_path) != built["engine_sha256"]:
        raise ValueError("artifact hash differs from stage report")
    model, names, version = prepared_model(args)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(onnx_path), sess_options=options, providers=["CPUExecutionProvider"])
    runner = EngineRunner(engine_path)
    if runner.metadata["names"] != names or runner.metadata["source_sha256"] != args.expected_sha256:
        raise ValueError("engine metadata/source mismatch")
    rng = np.random.default_rng(20261029)
    samples = [("synthetic_black", np.zeros((1, 3, 640, 640), dtype=np.float32)),
               ("synthetic_white", np.ones((1, 3, 640, 640), dtype=np.float32)),
               ("synthetic_random_seed_20261029", rng.random((1, 3, 640, 640), dtype=np.float32))]
    for path in args.sample:
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"cannot decode sample: {path}")
        samples.append((str(path), letterbox(image)))
    case_results, baselines = [], []
    for label, pixels in samples:
        with torch.inference_mode():
            reference = model(torch.from_numpy(pixels).cuda()).cpu().numpy()
        onnx_output = session.run(["output0"], {"images": pixels})[0]
        engine_output, gpu_ms, wall_ms = runner(pixels)
        case_results.append({"sample": label,
                             "sample_tensor_sha256": hashlib.sha256(pixels.tobytes()).hexdigest(),
                             "max_reference_confidence": float(reference[..., 4].max()),
                             "pt_vs_onnx": compare_outputs(reference, onnx_output,
                                                           confidence_tolerance=0.0001, box_tolerance=0.25),
                             "pt_vs_trt_fp16": compare_outputs(reference, engine_output,
                                                               confidence_tolerance=0.03, box_tolerance=3.0),
                             "gpu_execution_ms": gpu_ms, "copy_execute_return_ms": wall_ms})
        baselines.append(engine_output)
    for _ in range(10):
        runner(samples[0][1])
    gpu_times, wall_times, repeat_error = [], [], 0.0
    for index in range(100):
        sample_index = index % len(samples)
        output, gpu_ms, wall_ms = runner(samples[sample_index][1])
        check_output(output)
        repeat_error = max(repeat_error, float(np.abs(output - baselines[sample_index]).max()))
        gpu_times.append(gpu_ms)
        wall_times.append(wall_ms)
    passed = all(c[k]["status"] == "PASS" for c in case_results for k in ("pt_vs_onnx", "pt_vs_trt_fp16"))
    passed = passed and repeat_error <= 1e-5
    def distribution(values):
        return {"mean": float(np.mean(values)), "p50": float(np.percentile(values, 50)),
                "p95": float(np.percentile(values, 95)), "p99": float(np.percentile(values, 99)),
                "min": float(np.min(values)), "max": float(np.max(values))}
    report = {
        "status": "PASS" if passed else "FAIL", "recorded_at_device_utc": now_utc(),
        "cases": case_results, "continuous_inferences": 100, "warmup_inferences": 10,
        "repeat_max_absolute_error": repeat_error, "gpu_execution_ms": distribution(gpu_times),
        "copy_execute_return_ms": distribution(wall_times), "onnxruntime": ort.__version__,
        "onnxruntime_providers": session.get_providers(), "tensorrt": trt.__version__,
        "torch": torch.__version__, "cuda": torch.version.cuda,
        "gpu": torch.cuda.get_device_name(0), "uid": os.getuid(),
        "real_traffic_light_accuracy_validated": False,
        "validation_scope": "synthetic inputs and unlabeled existing images; not traffic-light gold samples",
        "topk_comparison": "all 300 sorted confidence scores; class/box matching only at confidence >= 0.5",
        "legacy_version": version,
    }
    write_json(args.output_dir / "verification_report.json", report)
    if not passed:
        print(json.dumps(report), flush=True)
        raise RuntimeError("numerical or continuity verification failed")
    provenance_path = args.output_dir / "run_provenance.json"
    provenance = json.loads(provenance_path.read_text()) if provenance_path.is_file() else {}
    manifest = {
        "model_version": "tl-20250724-trt107-fp16-20261029-candidate",
        "status": "candidate_requires_labeled_validation", "source_file": args.source.name,
        "source_sha256": args.expected_sha256, "onnx_file": onnx_path.name,
        "onnx_sha256": sha256(onnx_path), "engine_file": engine_path.name,
        "engine_sha256": sha256(engine_path), "classes": names,
        "architecture": "YOLOv10n", "legacy_ultralytics_version": version,
        "input_name": "images", "input_shape": [1, 3, 640, 640], "input_dtype": "float32",
        "preprocess": {"color": "RGB", "layout": "NCHW", "normalization": "uint8 / 255.0",
                       "resize": "aspect-preserving centered letterbox", "padding_uint8": 114,
                       "roi": "caller selects ROI before letterbox and retains coordinate transform"},
        "output_name": "output0", "output_shape": [1, 300, 6], "output_dtype": "float32",
        "output_columns": ["x1", "y1", "x2", "y2", "confidence", "class_id"],
        "output_coordinates": "pixels in the 640x640 letterboxed input",
        "postprocess": "YOLOv10 one2one TopK already in graph; filter confidence, undo letterbox/ROI; no extra NMS",
        "confidence_probe_threshold": 0.5, "production_confidence_threshold": None,
        "precision": "fp16_enabled_mixed_precision_with_fp32_io", "opset": 13,
        "tensorrt": trt.__version__, "cuda": torch.version.cuda,
        "l4t": built["l4t"], "gpu": torch.cuda.get_device_name(0),
        "workspace_mib": built["workspace_mib"], "engine_container": built["engine_container"],
        "source_code_sha256": exported["legacy_files_sha256"],
        "build_script_sha256": built.get("build_script_sha256")
        or provenance.get("export_build_script_sha256"),
        "verification_script_sha256": sha256(Path(__file__)),
        "build_commands": json.loads((args.output_dir.parent / "build_commands.json").read_text())
        if (args.output_dir.parent / "build_commands.json").is_file() else None,
        "real_traffic_light_accuracy_validated": False, "automatic_release_enabled": False,
        "continuous_inferences_passed": 100, "device_clock_known_incorrect": True,
        "recorded_at_device_utc": now_utc(),
    }
    write_json(args.output_dir / "model_manifest.json", manifest)
    print(json.dumps({k: v for k, v in report.items() if k != "cases"}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=["export", "build", "verify"])
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--legacy-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--workspace-mib", type=int, default=1024)
    parser.add_argument("--sample", type=Path, action="append", default=[])
    args = parser.parse_args()
    if sha256(args.source) != args.expected_sha256:
        raise ValueError("source checkpoint hash mismatch")
    if not 64 <= args.workspace_mib <= 2048:
        raise ValueError("workspace must be between 64 and 2048 MiB")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.stage == "export":
        export_onnx(args)
    elif args.stage == "build":
        build_engine(args)
    else:
        verify_model(args)


if __name__ == "__main__":
    main()
