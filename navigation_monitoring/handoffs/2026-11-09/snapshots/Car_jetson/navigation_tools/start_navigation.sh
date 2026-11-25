#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="${ROOT_DIR}/navigation_monitoring/runs/${RUN_ID}"

find_monitor_pids() {
    local proc_dir pid
    local -a argv
    local target="${ROOT_DIR}/navigation_monitoring/nav_task_monitor.py"
    for proc_dir in /proc/[0-9]*; do
        [[ -r "${proc_dir}/cmdline" ]] || continue
        pid="${proc_dir##*/}"
        [[ "${pid}" == "$$" || "${pid}" == "${PPID}" ]] && continue
        argv=()
        mapfile -d '' -t argv <"${proc_dir}/cmdline" 2>/dev/null || true
        if ((${#argv[@]} >= 2)) && [[ "${argv[1]}" == "${target}" ]]; then
            printf '%s\n' "${pid}"
        fi
    done
}

mapfile -t existing_monitor_pids < <(find_monitor_pids)
if ((${#existing_monitor_pids[@]} > 0)); then
    printf '发现已有导航监测器 PID: %s。为避免复用旧日志或无保护监测器，本次不派单。\n' \
        "${existing_monitor_pids[*]}" >&2
    printf '请先核实该进程对应任务已结束，停止并确认旧监测器退出，再重新启动本脚本。\n' >&2
    exit 2
fi

mkdir -p "${RUN_DIR}"
printf '%s\n' "${RUN_DIR}" >"${ROOT_DIR}/navigation_monitoring/current_run_dir"

source_if_present() {
    local setup_file="$1"
    if [[ -f "${setup_file}" ]]; then
        # shellcheck disable=SC1090
        set +u
        source "${setup_file}"
        set -u
    fi
}

set +u
source /opt/ros/humble/setup.bash
set -u
source_if_present "${ROOT_DIR}/real_ws/install/setup.bash"
source_if_present "${ROOT_DIR}/utils/install/setup.bash"
source_if_present "${ROOT_DIR}/slam/install/setup.bash"
source_if_present "${ROOT_DIR}/nav2/install/setup.bash"
source_if_present "${ROOT_DIR}/multi_map_navigation/install/setup.bash"

node_exists() {
    ros2 node list 2>/dev/null | rg -qx "$1"
}

management_stack_ready() {
    node_exists "/navigation_manager" \
        && node_exists "/route_planner" \
        && node_exists "/process_manager"
}

management_stack_running() {
    pgrep -f "[/]ros2 launch multi_map_navigation multi_map_navigation.launch.py" >/dev/null
}

if ! management_stack_ready; then
    # ROS discovery can lag behind a freshly launched process. Treat the launch
    # process as authoritative so a second invocation cannot create duplicate nodes.
    if ! management_stack_running; then
        setsid ros2 launch multi_map_navigation multi_map_navigation.launch.py \
            >"${RUN_DIR}/management_stack.log" 2>&1 < /dev/null &
        echo "$!" >"${RUN_DIR}/management_stack.pid"
    fi

    deadline=$((SECONDS + 45))
    until management_stack_ready; do
        if (( SECONDS >= deadline )); then
            if [[ -f "${RUN_DIR}/management_stack.log" ]]; then
                echo "导航管理栈未在45秒内就绪，查看 ${RUN_DIR}/management_stack.log" >&2
            else
                echo "已有导航管理栈进程，但节点未在45秒内就绪" >&2
            fi
            exit 2
        fi
        sleep 1
    done
fi

monitor_args=(--output "${RUN_DIR}/monitor.raw.jsonl")
monitor_protection="${NAV_MONITOR_ENABLE_PROTECTION:-1}"
if [[ "${monitor_protection}" == "1" ]]; then
    monitor_args+=(--enable-protection)
fi
setsid python3 "${ROOT_DIR}/navigation_monitoring/nav_task_monitor.py" \
    "${monitor_args[@]}" >"${RUN_DIR}/monitor.log" 2>&1 < /dev/null &
echo "$!" >"${RUN_DIR}/monitor.pid"

monitor_ready() {
    python3 - "${RUN_DIR}/monitor.raw.jsonl" "${ROOT_DIR}/navigation_monitoring/nav_task_monitor.py" \
        "${monitor_protection}" <<'PY'
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

output_path, script_path, protection = sys.argv[1:]
try:
    records = [json.loads(line) for line in Path(output_path).read_text(encoding="utf-8").splitlines()]
except (OSError, ValueError):
    raise SystemExit(1)

started = next((item for item in records if item.get("event") == "monitor_started"), None)
if not started or bool(started.get("protection_enabled")) != (protection == "1"):
    raise SystemExit(1)
try:
    argv = Path(f"/proc/{int(started['pid'])}/cmdline").read_bytes().split(b"\0")
except (OSError, KeyError, ValueError):
    raise SystemExit(1)
if len(argv) < 2 or argv[1] != os.fsencode(script_path):
    raise SystemExit(1)

snapshots = [item for item in records if item.get("event") == "snapshot"]
if not snapshots:
    raise SystemExit(1)
try:
    snapshot_age = (datetime.now(timezone.utc) - datetime.fromisoformat(snapshots[-1]["time"])).total_seconds()
except (KeyError, TypeError, ValueError):
    raise SystemExit(1)
if not 0 <= snapshot_age <= 8.0:
    raise SystemExit(1)
freshness = snapshots[-1].get("freshness_sec", {})
if any(freshness.get(name) is None or freshness[name] > 2.5
       for name in ("imu", "scan", "vehicle_status")):
    raise SystemExit(1)
PY
}

deadline=$((SECONDS + 15))
until node_exists "/navigation_task_monitor" && monitor_ready; do
    if ((SECONDS >= deadline)); then
        echo "本次监测器未就绪，或 IMU、合并激光、底盘状态数据不新鲜；未派发导航任务。查看 ${RUN_DIR}/monitor.log 和 ${RUN_DIR}/monitor.raw.jsonl" >&2
        exit 2
    fi
    sleep 1
done

python3 "${ROOT_DIR}/navigation_tools/dispatch_navigation.py" dispatch "$@"
echo "本次监测目录: ${RUN_DIR}"
