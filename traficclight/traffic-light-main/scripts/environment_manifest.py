#!/usr/bin/env python3
"""Describe inherited JetPack packages without changing the environment."""
import datetime
import grp
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import pwd
import site
import subprocess
import sys


def main():
    modules = {}
    for name in ("numpy", "cv2", "torch", "torchvision", "tensorrt", "gi"):
        try:
            module = importlib.import_module(name)
            path = Path(module.__file__)
            modules[name] = {
                "version": module.__version__,
                "path": str(path),
                "entry_file_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        except Exception as exc:
            modules[name] = {"error": f"{type(exc).__name__}: {exc}"}
    import cv2
    import torch
    import torchvision
    boxes = torch.tensor([[0, 0, 1, 1], [0, 0, 1, 1]], dtype=torch.float32, device="cuda")
    scores = torch.tensor([0.9, 0.5], device="cuda")
    nms_result = torchvision.ops.nms(boxes, scores, 0.5).tolist()
    if nms_result != [0]:
        raise RuntimeError(f"Unexpected torchvision CUDA NMS result: {nms_result}")
    packages = sorted(
        ({"name": dist.metadata["Name"], "version": dist.version}
         for dist in importlib.metadata.distributions()),
        key=lambda item: item["name"].lower(),
    )
    dpkg = subprocess.run(
        ["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\n",
         "nvidia-l4t-*", "cuda-*", "libnvinfer*", "python3-libnvinfer*"],
        capture_output=True, text=True, check=False, timeout=20,
    )
    report = {
        "schema_version": 1,
        "recorded_at_remote_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "hostname": platform.node(),
        "platform": platform.platform(),
        "l4t": Path("/etc/nv_tegra_release").read_text().strip(),
        "python": {"version": platform.python_version(), "executable": sys.executable,
                   "prefix": sys.prefix, "base_prefix": sys.base_prefix,
                   "user_site_enabled": site.ENABLE_USER_SITE, "sys_path": sys.path},
        "identity": {"uid": os.getuid(), "user": pwd.getpwuid(os.getuid()).pw_name,
                     "groups": [grp.getgrgid(g).gr_name for g in os.getgroups()]},
        "modules": modules,
        "cuda": {"available": torch.cuda.is_available(), "version": torch.version.cuda,
                 "device": torch.cuda.get_device_name(0), "torchvision_nms": nms_result},
        "opencv_gstreamer_build": [line.strip() for line in cv2.getBuildInformation().splitlines()
                                   if "GStreamer:" in line],
        "python_packages": packages,
        "jetpack_deb_packages": dpkg.stdout.splitlines(),
        "jetpack_dpkg_stderr": dpkg.stderr.strip(),
        "dependency_scope": "P2 diagnostics inherit system packages; requirements.lock adds no PyPI packages",
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
