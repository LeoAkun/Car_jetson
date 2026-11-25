#!/usr/bin/env bash
set -euo pipefail

if (( EUID != 0 )); then
    echo "Run this installer with sudo." >&2
    exit 1
fi

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
camera_path=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:4.1:1.0-video-index0
rule_source="$project_root/deploy/99-front-traffic-camera.rules"
rule_target=/etc/udev/rules.d/99-front-traffic-camera.rules

python3 - "$camera_path" <<'PY'
from pathlib import Path
import subprocess
import sys

path = Path(sys.argv[1])
if not path.exists():
    raise SystemExit(f"Confirmed camera is absent: {path}")
properties = subprocess.check_output(
    ["udevadm", "info", "--query=property", "--name=" + str(path)], text=True
)
values = dict(line.split("=", 1) for line in properties.splitlines() if "=" in line)
expected = {
    "ID_VENDOR_ID": "1bcf",
    "ID_MODEL_ID": "0001",
    "ID_PATH": "pci-0000:00:14.0-usb-0:4.1:1.0",
}
if any(values.get(key) != value for key, value in expected.items()):
    raise SystemExit("Camera identity changed; repeat physical confirmation before installing.")
index = Path("/sys/class/video4linux") / path.resolve().name / "index"
if index.read_text().strip() != "0":
    raise SystemExit("Refusing to bind a non-capture device index.")
print("Verified camera:", path.resolve())
PY

if [[ -e "$rule_target" ]] && ! cmp -s -- "$rule_source" "$rule_target"; then
    echo "A different rule already exists at $rule_target; inspect it before replacing." >&2
    exit 1
fi
if [[ -e /dev/front_traffic_camera || -L /dev/front_traffic_camera ]]; then
    if [[ "$(readlink -f /dev/front_traffic_camera)" != "$(readlink -f "$camera_path")" ]]; then
        echo "The camera alias already points elsewhere; refusing to replace it." >&2
        exit 1
    fi
fi

install -o root -g root -m 0644 -- "$rule_source" "$rule_target"
udevadm control --reload-rules
device_name="$(basename -- "$(readlink -f -- "$camera_path")")"
udevadm trigger --action=change --subsystem-match=video4linux --sysname-match="$device_name"
udevadm settle --timeout=10

if [[ "$(readlink -f /dev/front_traffic_camera)" != "$(readlink -f "$camera_path")" ]]; then
    echo "Rule installed, but alias verification failed." >&2
    exit 1
fi
ls -l /dev/front_traffic_camera
echo "Installed persistent front camera mapping. Keep the camera on this USB port."
