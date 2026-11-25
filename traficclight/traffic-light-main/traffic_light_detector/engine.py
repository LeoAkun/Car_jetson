"""TensorRT 10 runner using the existing JetPack PyTorch CUDA allocator."""
from __future__ import annotations

from pathlib import Path
import threading

import numpy as np

from .contract import INPUT_SHAPE, OUTPUT_SHAPE, load_candidate, validate_io


class TensorRTRunner:
    def __init__(self, engine_path: Path, manifest_path: Path, expected_sha256: str):
        self.manifest, self.metadata, serialized = load_candidate(
            engine_path, manifest_path, expected_sha256)
        # Keep CUDA/TRT imports out of the CPU/mock path. Never import Ultralytics.
        import tensorrt as trt
        import torch

        if trt.__version__ != self.manifest["tensorrt"] or not trt.__version__.startswith("10."):
            raise RuntimeError("TensorRT runtime differs from candidate build")
        self.model_sha256 = expected_sha256
        self.model_version = self.manifest["model_version"]
        self.logger = trt.Logger(trt.Logger.ERROR)
        trt.init_libnvinfer_plugins(self.logger, "")
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(serialized)
        if self.engine is None:
            raise RuntimeError("engine deserialization failed")
        validate_io([
            (name, str(self.engine.get_tensor_mode(name)).split(".")[-1],
             self.engine.get_tensor_shape(name), str(np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))))
            for name in (self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors))
        ])
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("engine context creation failed")
        self.buffers = {name: torch.empty(shape, device="cuda", dtype=torch.float32)
                        for name, shape in (("images", INPUT_SHAPE), ("output0", OUTPUT_SHAPE))}
        for name, value in self.buffers.items():
            if not self.context.set_tensor_address(name, value.data_ptr()):
                raise RuntimeError(f"cannot bind {name}")
        self.stream = torch.cuda.Stream()
        self.lock = threading.Lock()
        self.failed = False

    def infer(self, pixels):
        import torch

        if (not isinstance(pixels, np.ndarray) or pixels.shape != INPUT_SHAPE
                or pixels.dtype != np.float32 or not pixels.flags.c_contiguous
                or not np.isfinite(pixels).all() or pixels.min() < 0 or pixels.max() > 1):
            raise ValueError("invalid inference input")
        with self.lock:
            if self.failed:
                raise RuntimeError("runner requires restart after GPU failure")
            try:
                with torch.cuda.stream(self.stream):
                    self.buffers["images"].copy_(torch.from_numpy(pixels))
                    if not self.context.execute_async_v3(self.stream.cuda_stream):
                        raise RuntimeError("TensorRT execution failed")
                    # Copy on this same stream, then synchronize before returning ownership.
                    output = self.buffers["output0"].cpu()
                self.stream.synchronize()
                return output.numpy().copy()
            except Exception:
                self.failed = True
                raise
