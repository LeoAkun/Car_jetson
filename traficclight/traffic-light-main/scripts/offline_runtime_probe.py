#!/usr/bin/env python3
"""Exercise image decoding and a tiny TensorRT network without a traffic model."""
import argparse
import json
import os
from pathlib import Path
import socket

import cv2
import numpy as np
import tensorrt as trt
import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Cannot decode {args.image}")
    pixels = cv2.resize(image, (32, 32)).transpose(2, 0, 1)[None]
    pixels = np.ascontiguousarray(pixels, dtype=np.float32) / 255.0
    inputs = torch.from_numpy(pixels).to(device="cuda").contiguous()
    outputs = torch.empty_like(inputs)
    logger = trt.Logger(trt.Logger.ERROR)
    builder = trt.Builder(logger)
    network = builder.create_network(0)
    tensor = network.add_input("pixels", trt.float32, tuple(inputs.shape))
    layer = network.add_identity(tensor)
    layer.get_output(0).name = "copy"
    network.mark_output(layer.get_output(0))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 64 * 1024 * 1024)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT identity build failed")
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None:
        raise RuntimeError("TensorRT identity deserialize failed")
    context = engine.create_execution_context()
    for name, value in (("pixels", inputs), ("copy", outputs)):
        if not context.set_tensor_address(name, value.data_ptr()):
            raise RuntimeError(f"Cannot bind {name}")
    stream = torch.cuda.current_stream()
    if not context.execute_async_v3(stream.cuda_stream):
        raise RuntimeError("TensorRT identity execution failed")
    stream.synchronize()
    max_error = float((outputs - inputs).abs().max().item())
    if max_error != 0:
        raise RuntimeError(f"TensorRT output differs: {max_error}")
    print(json.dumps({
        "overall": "PASS",
        "uid": os.getuid(),
        "interfaces": socket.if_nameindex(),
        "image_shape": list(image.shape),
        "gpu": torch.cuda.get_device_name(0),
        "tensorrt": trt.__version__,
        "probe": "in-memory identity network; no traffic-light weights",
        "max_absolute_error": max_error,
    }, indent=2))


if __name__ == "__main__":
    main()
