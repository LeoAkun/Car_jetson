#!/usr/bin/env python3
"""Read-only Jetson runtime checks for the traffic-light detector."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable


@dataclass
class CheckResult:
    name: str
    status: str
    required: bool
    detail: str
    data: dict[str, Any] | None = None


def result(
    name: str,
    ok: bool,
    detail: str,
    *,
    required: bool = True,
    data: dict[str, Any] | None = None,
) -> CheckResult:
    return CheckResult(
        name=name,
        status="PASS" if ok else ("FAIL" if required else "WARN"),
        required=required,
        detail=detail,
        data=data,
    )


def check_python() -> CheckResult:
    version = platform.python_version()
    ok = sys.version_info >= (3, 10)
    return result(
        "python",
        ok,
        f"Python {version} at {sys.executable}",
        data={"version": version, "executable": sys.executable},
    )


def check_l4t() -> CheckResult:
    release = Path("/etc/nv_tegra_release")
    if not release.is_file():
        return result("l4t", False, f"missing {release}")
    first_line = release.read_text(encoding="utf-8", errors="replace").splitlines()[0]
    return result("l4t", bool(first_line.strip()), first_line.strip())


def import_module(name: str) -> tuple[Any | None, str | None]:
    try:
        return importlib.import_module(name), None
    except Exception as exc:  # diagnostic boundary
        return None, f"{type(exc).__name__}: {exc}"


def check_opencv(image: Path | None) -> CheckResult:
    cv2, error = import_module("cv2")
    if error:
        return result("opencv", False, error)

    data: dict[str, Any] = {"version": cv2.__version__}
    if image is not None:
        decoded = cv2.imread(str(image), cv2.IMREAD_COLOR)
        if decoded is None:
            return result(
                "opencv",
                False,
                f"OpenCV {cv2.__version__}; failed to decode {image}",
                data=data,
            )
        data.update(
            {
                "image": str(image),
                "shape": list(decoded.shape),
                "min": int(decoded.min()),
                "max": int(decoded.max()),
                "mean": round(float(decoded.mean()), 3),
            }
        )
    return result("opencv", True, f"OpenCV {cv2.__version__}", data=data)


def check_torch() -> CheckResult:
    torch, error = import_module("torch")
    if error:
        return result("torch_cuda", False, error)

    cuda_available = bool(torch.cuda.is_available())
    data: dict[str, Any] = {
        "version": torch.__version__,
        "cuda_available": cuda_available,
        "torch_cuda_version": torch.version.cuda,
    }
    if cuda_available:
        try:
            data["device_name"] = torch.cuda.get_device_name(0)
            probe = torch.ones((16, 16), device="cuda")
            data["probe_sum"] = float(probe.sum().item())
            del probe
        except Exception as exc:  # diagnostic boundary
            return result(
                "torch_cuda",
                False,
                f"CUDA probe failed: {type(exc).__name__}: {exc}",
                data=data,
            )
    return result(
        "torch_cuda",
        cuda_available,
        f"torch {torch.__version__}; CUDA available={cuda_available}",
        data=data,
    )


def read_engine_file(path: Path) -> tuple[bytes, dict[str, Any] | None]:
    """Accept raw TensorRT plans and Ultralytics length-prefixed JSON wrappers."""
    serialized = path.read_bytes()
    if len(serialized) > 4:
        metadata_size = int.from_bytes(serialized[:4], "little")
        if 0 < metadata_size <= 1024 * 1024 and 4 + metadata_size < len(serialized):
            try:
                metadata = json.loads(serialized[4 : 4 + metadata_size].decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
            else:
                if isinstance(metadata, dict):
                    return serialized[4 + metadata_size :], metadata
    return serialized, None


def load_engine_for_check(
    path: Path,
    manifest_path: Path | None,
    expected_sha256: str | None,
) -> tuple[bytes, dict[str, Any] | None, dict[str, Any] | None]:
    """Use the P4 contract for a pinned candidate; keep raw-plan diagnostics generic."""
    if manifest_path is None:
        serialized, metadata = read_engine_file(path)
        return serialized, metadata, None
    if expected_sha256 is None:
        raise ValueError("--expected-sha256 is required with --manifest")
    source_root = str(Path(__file__).resolve().parents[1])
    if source_root not in sys.path:
        sys.path.insert(0, source_root)
    from traffic_light_detector.contract import load_candidate

    manifest, metadata, serialized = load_candidate(path, manifest_path, expected_sha256)
    return serialized, metadata, manifest


def check_tensorrt(
    engine_paths: list[Path],
    manifest_path: Path | None = None,
    expected_sha256: str | None = None,
) -> CheckResult:
    candidate = None
    if manifest_path is not None:
        if expected_sha256 is None or len(engine_paths) != 1:
            return result("tensorrt", False, "candidate check requires one engine and an independent SHA-256")
        candidate_path = engine_paths[0]
        if not candidate_path.is_file():
            return result("tensorrt", False, f"{candidate_path}: file not found")
        try:
            candidate = load_engine_for_check(candidate_path, manifest_path, expected_sha256)
        except Exception as exc:
            return result(
                "tensorrt", False,
                f"candidate contract failed before TensorRT import: {type(exc).__name__}: {exc}",
            )

    trt, error = import_module("tensorrt")
    if error:
        return result("tensorrt", False, error)

    data: dict[str, Any] = {"version": trt.__version__, "engines": []}
    failures: list[str] = []
    for path in engine_paths:
        engine_data: dict[str, Any] = {"path": str(path)}
        if not path.is_file():
            engine_data["error"] = "file not found"
            failures.append(f"{path}: file not found")
            data["engines"].append(engine_data)
            continue
        try:
            logger = trt.Logger(trt.Logger.ERROR)
            trt.init_libnvinfer_plugins(logger, "")
            runtime = trt.Runtime(logger)
            if candidate is not None:
                serialized, metadata, manifest = candidate
            else:
                serialized, metadata, manifest = load_engine_for_check(path, None, None)
            engine_data["metadata_header_bytes"] = path.stat().st_size - len(serialized)
            if metadata is not None:
                engine_data["metadata"] = metadata
            if manifest is not None:
                engine_data["candidate_contract"] = "validated"
                engine_data["engine_sha256"] = expected_sha256
                engine_data["model_version"] = manifest["model_version"]
                if trt.__version__ != manifest["tensorrt"]:
                    raise RuntimeError(
                        "TensorRT runtime differs from pinned candidate manifest: "
                        f"{trt.__version__} != {manifest['tensorrt']}"
                    )
            engine = runtime.deserialize_cuda_engine(serialized)
            if engine is None:
                raise RuntimeError("deserialize_cuda_engine returned None")
            engine_data["deserialized"] = True
            engine_data["io_tensors"] = []
            contract_tensors = []
            for index in range(engine.num_io_tensors):
                name = engine.get_tensor_name(index)
                mode = str(engine.get_tensor_mode(name)).split(".")[-1].upper()
                dtype = str(engine.get_tensor_dtype(name))
                shape = list(engine.get_tensor_shape(name))
                engine_data["io_tensors"].append(
                    {
                        "name": name,
                        "mode": mode,
                        "dtype": dtype,
                        "shape": shape,
                    }
                )
                contract_dtype = dtype.split(".")[-1].lower()
                if manifest is not None:
                    tensor_dtype = trt.nptype(engine.get_tensor_dtype(name))
                    contract_dtype = getattr(tensor_dtype, "__name__", str(tensor_dtype)).lower()
                contract_tensors.append((name, mode, shape, contract_dtype))
            if manifest is not None:
                from traffic_light_detector.contract import validate_io

                validate_io(contract_tensors)
                engine_data["io_contract"] = "validated"
        except Exception as exc:  # diagnostic boundary
            engine_data["deserialized"] = False
            engine_data["error"] = f"{type(exc).__name__}: {exc}"
            failures.append(f"{path}: {engine_data['error']}")
        data["engines"].append(engine_data)

    ok = not failures
    detail = f"TensorRT {trt.__version__}"
    if engine_paths:
        detail += f"; engines {len(engine_paths) - len(failures)}/{len(engine_paths)} loadable"
    return result("tensorrt", ok, detail if ok else "; ".join(failures), data=data)


def run_command(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )


def check_gstreamer(elements: list[str]) -> CheckResult:
    binary = shutil.which("gst-inspect-1.0")
    if binary is None:
        return result("gstreamer", False, "gst-inspect-1.0 not found")

    missing: list[str] = []
    for element in elements:
        completed = run_command([binary, element])
        if completed.returncode != 0:
            missing.append(element)
    return result(
        "gstreamer",
        not missing,
        "all required elements found" if not missing else f"missing: {', '.join(missing)}",
        data={"required_elements": elements, "missing": missing},
    )


def check_ports(bind_host: str, ports: list[int]) -> CheckResult:
    unavailable: list[str] = []
    checked: list[str] = []
    for port in ports:
        endpoint = f"{bind_host}:{port}"
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((bind_host, port))
            checked.append(endpoint)
        except OSError as exc:
            unavailable.append(f"{endpoint}: {exc}")
        finally:
            sock.close()
    return result(
        "ports",
        not unavailable,
        "available: " + ", ".join(checked) if not unavailable else "; ".join(unavailable),
        data={"checked": checked, "unavailable": unavailable},
    )


def check_disk(path: Path, minimum_free_gib: float) -> CheckResult:
    target = path if path.exists() else path.parent
    usage = shutil.disk_usage(target)
    free_gib = usage.free / (1024**3)
    return result(
        "disk",
        free_gib >= minimum_free_gib,
        f"{free_gib:.2f} GiB free at {target}; minimum {minimum_free_gib:.2f} GiB",
        data={"path": str(target), "free_gib": round(free_gib, 3)},
    )


def check_manifest(path: Path | None) -> CheckResult:
    if path is None:
        return result(
            "model_manifest",
            False,
            "not supplied; required by P3 before real inference",
            required=False,
        )
    if not path.is_file():
        return result("model_manifest", False, f"file not found: {path}", required=False)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return result(
            "model_manifest",
            False,
            f"invalid JSON: {type(exc).__name__}: {exc}",
            required=False,
        )
    required_keys = {"model_version", "engine_sha256", "source_sha256", "classes", "input_shape"}
    missing = sorted(required_keys - payload.keys())
    return result(
        "model_manifest",
        not missing,
        "valid required fields" if not missing else f"missing keys: {', '.join(missing)}",
        required=False,
        data={"path": str(path), "missing": missing},
    )


def build_checks(args: argparse.Namespace) -> list[Callable[[], CheckResult]]:
    return [
        check_python,
        check_l4t,
        lambda: check_opencv(args.image),
        check_torch,
        lambda: check_tensorrt(args.engine, args.manifest, args.expected_sha256),
        lambda: check_gstreamer(args.gst_element),
        lambda: check_ports(args.bind_host, args.port),
        lambda: check_disk(args.disk_path, args.minimum_free_gib),
        lambda: check_manifest(args.manifest),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="emit one JSON document")
    parser.add_argument("--image", type=Path, help="offline JPEG/PNG to decode with OpenCV")
    parser.add_argument("--engine", type=Path, action="append", default=[], help="TensorRT engine to deserialize")
    parser.add_argument("--manifest", type=Path, help="P3 candidate manifest; requires one engine and an independent SHA-256 pin")
    parser.add_argument("--expected-sha256", help="independently pinned candidate engine SHA-256")
    parser.add_argument("--bind-host", default="192.168.1.150", help="local address used for port availability checks")
    parser.add_argument("--port", type=int, action="append", default=[], help="TCP port to check; default: 5000 and 12345")
    parser.add_argument("--disk-path", type=Path, default=Path("/opt/traffic-light"))
    parser.add_argument("--minimum-free-gib", type=float, default=10.0)
    parser.add_argument(
        "--gst-element",
        action="append",
        default=[],
        help="required GStreamer element; defaults cover the TCP MJPEG receiver",
    )
    args = parser.parse_args()
    if args.manifest is not None and len(args.engine) != 1:
        parser.error("--manifest requires exactly one --engine")
    if (args.manifest is None) != (args.expected_sha256 is None):
        parser.error("--manifest and --expected-sha256 must be supplied together")
    if not args.port:
        args.port = [5000, 12345]
    if not args.gst_element:
        args.gst_element = ["tcpserversrc", "multipartdemux", "jpegdec", "appsink"]
    return args


def main() -> int:
    args = parse_args()
    checks: list[CheckResult] = []
    for check in build_checks(args):
        try:
            checks.append(check())
        except Exception as exc:  # keep reporting after one unexpected diagnostic failure
            checks.append(result("internal_check", False, f"{type(exc).__name__}: {exc}"))

    required_failures = [item for item in checks if item.required and item.status == "FAIL"]
    report = {
        "schema_version": 1,
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "architecture": platform.machine(),
        "overall": "PASS" if not required_failures else "FAIL",
        "checks": [asdict(item) for item in checks],
    }
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=True))
    else:
        for item in checks:
            print(f"[{item.status}] {item.name}: {item.detail}")
        print(f"OVERALL: {report['overall']}")
    return 0 if not required_failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
