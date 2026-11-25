#!/usr/bin/env bash
set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
current_boot=$(cat /proc/sys/kernel/random/boot_id)

printf 'Current boot ID: %s\n' "$current_boot"
printf 'Recent test runs:\n'
find "$root_dir/runs" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort -r | head -n 10

latest_state="$root_dir/latest_run/state.json"
if [[ ! -r "$latest_state" ]]; then
  printf 'No previous state file found.\n'
  exit 1
fi

python3 - "$latest_state" "$current_boot" <<'PY'
import json
import sys

path, current_boot = sys.argv[1:]
with open(path) as handle:
    state = json.load(handle)
print(json.dumps(state, ensure_ascii=False, indent=2))
if state.get("status") == "RUNNING" and state.get("boot_id") != current_boot:
    print("\n判定：测试未正常收尾且 boot ID 已变化，主机在测试中发生了重启/断电。")
elif state.get("status") == "RUNNING":
    print("\n判定：测试未正常收尾，但 boot ID 未变化，可能是脚本被强制终止。")
else:
    print("\n判定：最近一次测试已正常记录结束状态。")
PY

printf '\nPrevious-boot high-priority kernel messages (if persistent journal is available):\n'
journalctl -b -1 -k -p warning..alert --no-pager -n 100 2>&1 || true
