#!/usr/bin/env bash
# Create the P2 runtime without installing or replacing NVIDIA packages.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "Run with sudo on the Jetson." >&2
    exit 1
fi
if [[ $(uname -m) != aarch64 || ! -f /etc/nv_tegra_release ]]; then
    echo "This installer requires a Jetson aarch64 host." >&2
    exit 1
fi
/usr/bin/python3 -c 'import sys, venv, ensurepip; assert sys.version_info[:2] == (3, 10), sys.version'
command -v setfacl >/dev/null
installer_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
gpu_rule_source="$installer_dir/../deploy/99-trafficlight-gpu.rules"
if [[ ! -f $gpu_rule_source ]]; then
    echo "Missing $gpu_rule_source; keep scripts and deploy directories together." >&2
    exit 1
fi

runtime_root=/opt/traffic-light
runtime_user=trafficlight
if getent passwd "$runtime_user" >/dev/null; then
    IFS=: read -r _ _ runtime_uid _ _ runtime_home runtime_shell < <(getent passwd "$runtime_user")
    if [[ $runtime_uid -eq 0 || $runtime_home != "$runtime_root" || $runtime_shell != /usr/sbin/nologin ]]; then
        echo "Existing trafficlight user differs from the required service identity; stopping." >&2
        exit 1
    fi
else
    if [[ -e $runtime_root || -L $runtime_root ]]; then
        echo "$runtime_root already exists without the service user; stopping." >&2
        exit 1
    fi
    useradd --system --user-group --home-dir "$runtime_root" --shell /usr/sbin/nologin "$runtime_user"
fi

for path in "$runtime_root" "$runtime_root/releases" "$runtime_root/shared" \
    "$runtime_root/shared/config" "$runtime_root/shared/recordings" \
    "$runtime_root/shared/logs" "$runtime_root/shared/cache"; do
    if [[ -L $path ]]; then
        echo "Refusing a symlink at $path" >&2
        exit 1
    fi
    if [[ -e $path && $(stat -c %U "$path") != "$runtime_user" ]]; then
        echo "Existing directory is not owned by $runtime_user: $path" >&2
        exit 1
    fi
    install -d -m 0750 -o "$runtime_user" -g "$runtime_user" "$path"
done

if [[ -L $runtime_root/venv ]]; then
    echo "Refusing a symlink at $runtime_root/venv" >&2
    exit 1
fi
if [[ ! -e $runtime_root/venv ]]; then
    runuser -u "$runtime_user" -- /usr/bin/python3 -m venv --system-site-packages "$runtime_root/venv"
fi
runuser -u "$runtime_user" -- env PYTHONNOUSERSITE=1 \
    "$runtime_root/venv/bin/python" -c '
import site, sys
from pathlib import Path
assert sys.version_info[:2] == (3, 10), sys.version
assert sys.prefix == "/opt/traffic-light/venv", sys.prefix
assert sys.prefix != sys.base_prefix
assert "include-system-site-packages = true" in (Path(sys.prefix) / "pyvenv.cfg").read_text()
assert not site.ENABLE_USER_SITE
print("Runtime user, directories and Python 3.10 venv ready.")
'

gpu_rule_target=/etc/udev/rules.d/99-trafficlight-gpu.rules
if [[ -e $gpu_rule_target ]] && ! cmp -s "$gpu_rule_source" "$gpu_rule_target"; then
    echo "Existing GPU rule differs; stopping without replacing it." >&2
    exit 1
fi
install -m 0644 -o root -g root "$gpu_rule_source" "$gpu_rule_target"
udevadm control --reload-rules
for node in /dev/nvmap /dev/nvhost-as-gpu /dev/nvhost-ctrl-gpu /dev/nvhost-gpu \
    /dev/nvhost-tsg-gpu /dev/nvhost-power-gpu /dev/dri/renderD* \
    /dev/nvgpu/igpu0/as /dev/nvgpu/igpu0/channel /dev/nvgpu/igpu0/ctrl \
    /dev/nvgpu/igpu0/tsg /dev/nvgpu/igpu0/power; do
    if [[ -c $node ]]; then
        setfacl -m u:trafficlight:rw- "$node"
    fi
done
echo "GPU ACLs applied; persistent udev rule installed. No video group membership added."
