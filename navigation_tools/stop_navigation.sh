#!/usr/bin/env bash
# Stop the multi-map navigation stack without touching vehicle sensor drivers.

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHECK_ONLY=0
FORCE_KILL=1
GRACE_SECONDS=12
SERVICE_TIMEOUT=15

usage() {
    cat <<'EOF'
用法：
  navigation_tools/stop_navigation.sh [选项]

关闭顺序：
  1. 向 /process_manager/fault 发布一次 operator stop；
  2. 等待管理器取消活动目标并关闭 navigation2、LIO-SAM、初始位姿和重定位；
  3. 通过 shutdown_process 服务补充关闭仍在运行的子栈；
  4. 向 multi_map_navigation launch 发送 SIGINT；
  5. 对明确属于导航栈的残留依次发送 SIGINT、SIGTERM，必要时 SIGKILL；
  6. 关闭本次命令可能使用的ROS 2 CLI守护进程；
  7. 检查导航相关ROS节点、操作系统进程、Fast DDS文件、FD/mmap持有者及内核锁。

选项：
  --check-only       只检查，不发布ROS消息、不调用服务、不发送信号
  --no-force-kill    残留进程经过SIGINT和SIGTERM后仍存在时只报告，不发SIGKILL
  --grace SEC        每个进程退出阶段的等待秒数，默认12
  --service-timeout SEC
                     单次ROS服务调用超时，默认15
  -h, --help         显示帮助

退出码：
  0  未发现导航栈、ROS 2 CLI守护进程或独立Fast DDS discovery进程残留
  2  仍发现导航相关ROS节点、进程、CLI守护进程或Fast DDS discovery进程
  64 参数错误

说明：
  本脚本不会停止 launch_real、传感器、底盘驱动、rosbag、导航监测器或温度监控。
  ROS节点检查使用--no-daemon；停止CLI守护进程不会停止传感器节点中的DDS通信。
  传感器仍运行时不会删除/dev/shm中的Fast DDS共享内存文件。
  独立Fast DDS discovery进程只报告、不自动停止，避免影响传感器通信。
  Fast DDS文件存在本身不算故障；FD、mmap和/proc/locks占用会单独记录。
  每次运行会在 navigation_monitoring/shutdown_checks/<时间>/ 保存检查记录。
EOF
}

is_nonnegative_number() {
    [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]
}

while (($#)); do
    case "$1" in
        --check-only)
            CHECK_ONLY=1
            shift
            ;;
        --no-force-kill)
            FORCE_KILL=0
            shift
            ;;
        --grace)
            if (($# < 2)) || ! is_nonnegative_number "$2"; then
                echo "--grace 需要非负秒数" >&2
                exit 64
            fi
            GRACE_SECONDS="$2"
            shift 2
            ;;
        --service-timeout)
            if (($# < 2)) || ! is_nonnegative_number "$2"; then
                echo "--service-timeout 需要非负秒数" >&2
                exit 64
            fi
            SERVICE_TIMEOUT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数：$1" >&2
            usage >&2
            exit 64
            ;;
    esac
done

RUN_ID="$(date +%Y-%m-%d_%H-%M-%S)"
REPORT_DIR="${ROOT_DIR}/navigation_monitoring/shutdown_checks/${RUN_ID}"
mkdir -p "${REPORT_DIR}"
LOG_FILE="${REPORT_DIR}/stop_navigation.log"
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "导航栈收尾开始：$(date -Ins)"
echo "模式：$([[ ${CHECK_ONLY} -eq 1 ]] && echo 只检查 || echo 关闭并检查)"
echo "记录目录：${REPORT_DIR}"

# One generated setup file carries the complete prefix chain on this host.
if [[ -r "${ROOT_DIR}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${ROOT_DIR}/install/setup.bash"
    set -u
elif [[ -r /opt/ros/humble/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "错误：找不到python3，无法进行可靠的/proc身份检查" >&2
    exit 2
fi

inventory_processes() {
    python3 <<'PY'
import os
import pathlib

management_names = {
    'process_manager', 'mqtt_task_receiver', 'status_reporter',
    'route_planner', 'navigation_manager', 'map_switch_controller',
}
lio_names = {
    'lio_sam_imuPreintegration', 'lio_sam_imageProjection',
    'lio_sam_featureExtraction', 'lio_sam_mapOptimization',
}

def read_process(proc_dir):
    try:
        raw = (proc_dir / 'cmdline').read_bytes().split(b'\0')
        argv = [item.decode(errors='replace') for item in raw if item]
        stat_tail = (proc_dir / 'stat').read_text().split(') ', 1)[1].split()
        exe = os.readlink(proc_dir / 'exe')
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
        return None
    if not argv or stat_tail[0] == 'Z':
        return None
    return {
        'pid': int(proc_dir.name),
        'ppid': int(stat_tail[1]),
        'pgid': int(stat_tail[2]),
        'state': stat_tail[0],
        'start_ticks': stat_tail[19],
        'exe': exe,
        'argv': argv,
        'cmd': ' '.join(argv),
    }

def has_launch(argv, package, launch_file):
    for index in range(len(argv) - 3):
        if (argv[index].endswith('/ros2') or argv[index] == 'ros2') and argv[index + 1] == 'launch':
            return argv[index + 2] == package and os.path.basename(argv[index + 3]) == launch_file
    return False

def classify(item):
    argv = item['argv']
    cmd = item['cmd']
    exe = item['exe']
    basename = os.path.basename(exe)
    entrypoint_names = {
        os.path.basename(arg) for arg in argv
        if '/lib/' in arg and not arg.endswith('/')
    }

    if has_launch(argv, 'multi_map_navigation', 'multi_map_navigation.launch.py'):
        return 'management-launch'
    if any(os.path.basename(arg) == 'management_reuse_reporter.launch.py' for arg in argv):
        return 'management-launch'

    if ((basename in management_names and '/lib/multi_map_navigation/' in exe) or
            (any(name in management_names for name in entrypoint_names) and
             any('/lib/multi_map_navigation/' in arg for arg in argv))):
        return 'management-node'

    if (has_launch(argv, 're_localization', 'run.real_launch.py') or
            has_launch(argv, 'nav2_init_pose', 'run.real_launch.py') or
            has_launch(argv, 'lio_sam', 'run.real_launch.py') or
            has_launch(argv, 'nav2', 'run.real_launch.py')):
        return 'navigation-launch'

    if ((basename == 'nav2_init_pose' and '/nav2_init_pose/' in exe) or
            any(os.path.basename(arg) == 'nav2_init_pose' and
                '/lib/nav2_init_pose/' in arg for arg in argv) or
            'ros2 run nav2_init_pose nav2_init_pose' in cmd):
        return 'initial-pose'

    if basename == 'pcl_localization_node' and '/re_localization/' in exe:
        return 're-localization'
    if basename in lio_names and '/lio_sam/' in exe:
        return 'lio-sam'
    if basename == 'nav2_costmap_2d_cloud' and '/nav2_costmap_2d/' in exe:
        return 'navigation2'

    # These two robot_localization nodes are created only by this repository's
    # LIO-SAM launch and carry explicit, unique ROS node remaps.
    if basename == 'ekf_node' and '__node:=ekf_gps' in argv:
        return 'lio-sam'
    if basename == 'navsat_transform_node' and '__node:=navsat_transform' in argv:
        return 'lio-sam'

    if basename in {'component_container_isolated', 'component_container_mt'}:
        if any(arg in {'__node:=nav2_container', '__node:=re_localization_container'} for arg in argv):
            return 'navigation2'
        if any('/Car_jetson/nav2/' in arg for arg in argv):
            return 'navigation2'

    # RViz is included by the Nav2 and optional LIO-SAM launches. Only select an
    # instance carrying their config path; an unrelated operator RViz remains.
    if basename == 'rviz2' and any(
            '/Car_jetson/nav2/' in arg or
            ('/Car_jetson/slam/' in arg and '/lio_sam/' in arg)
            for arg in argv):
        return 'navigation-rviz'

    return None

for proc_dir in sorted(pathlib.Path('/proc').glob('[0-9]*'), key=lambda p: int(p.name)):
    item = read_process(proc_dir)
    if item is None:
        continue
    category = classify(item)
    if category is None:
        continue
    safe_cmd = item['cmd'].replace('\t', ' ').replace('\n', ' ')
    print(
        f"{category}\t{item['pid']}\t{item['ppid']}\t{item['pgid']}\t"
        f"{item['start_ticks']}\t{item['state']}\t{item['exe']}\t{safe_cmd}"
    )
PY
}

inventory_ros_cli_daemons() {
    python3 <<'PY'
import os
import pathlib

for proc_dir in sorted(pathlib.Path('/proc').glob('[0-9]*'), key=lambda p: int(p.name)):
    try:
        argv = [item.decode(errors='replace') for item in
                (proc_dir / 'cmdline').read_bytes().split(b'\0') if item]
        stat_tail = (proc_dir / 'stat').read_text().split(') ', 1)[1].split()
        exe = os.readlink(proc_dir / 'exe')
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
        continue
    if not argv or stat_tail[0] == 'Z':
        continue
    cmd = ' '.join(argv)
    if 'ros2cli.daemon.daemonize' not in cmd or 'ros2-daemon' not in argv:
        continue
    safe_cmd = cmd.replace('\t', ' ').replace('\n', ' ')
    print(
        f"ros2-cli-daemon\t{proc_dir.name}\t{stat_tail[1]}\t{stat_tail[2]}\t"
        f"{stat_tail[19]}\t{stat_tail[0]}\t{exe}\t{safe_cmd}"
    )
PY
}

inventory_fastdds_helpers() {
    python3 <<'PY'
import os
import pathlib

for proc_dir in sorted(pathlib.Path('/proc').glob('[0-9]*'), key=lambda p: int(p.name)):
    try:
        argv = [item.decode(errors='replace') for item in
                (proc_dir / 'cmdline').read_bytes().split(b'\0') if item]
        stat_tail = (proc_dir / 'stat').read_text().split(') ', 1)[1].split()
        exe = os.readlink(proc_dir / 'exe')
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
        continue
    if not argv or stat_tail[0] == 'Z':
        continue
    exe_name = os.path.basename(exe)
    arg_names = {os.path.basename(arg) for arg in argv}
    is_server = exe_name == 'fast-discovery-server' or 'fast-discovery-server' in arg_names
    is_cli_server = 'fastdds' in arg_names and 'discovery' in argv
    if not is_server and not is_cli_server:
        continue
    safe_cmd = ' '.join(argv).replace('\t', ' ').replace('\n', ' ')
    print(
        f"fastdds-discovery\t{proc_dir.name}\t{stat_tail[1]}\t{stat_tail[2]}\t"
        f"{stat_tail[19]}\t{stat_tail[0]}\t{exe}\t{safe_cmd}"
    )
PY
}

inventory_fastdds_shm() {
    find /dev/shm -maxdepth 1 \
        \( -name 'fastrtps*' -o -name 'fastdds*' \
           -o -name 'sem.fastrtps*' -o -name 'sem.fastdds*' \) \
        -printf '%f\n' 2>/dev/null | sort
}

inventory_fastdds_usage() {
    python3 <<'PY'
import os
import pathlib

shm_by_inode = {}
shm_paths = set()
for path in pathlib.Path('/dev/shm').glob('*'):
    if not path.name.startswith(('fastrtps', 'fastdds',
                                 'sem.fastrtps', 'sem.fastdds')):
        continue
    try:
        stat = path.stat()
    except OSError:
        continue
    resolved = str(path)
    shm_paths.add(resolved)
    shm_by_inode[(os.major(stat.st_dev), os.minor(stat.st_dev), stat.st_ino)] = resolved

records = set()
for proc_dir in pathlib.Path('/proc').glob('[0-9]*'):
    try:
        cmd = (proc_dir / 'cmdline').read_bytes().replace(
            b'\0', b' ').decode(errors='replace').strip()
    except OSError:
        continue
    pid = int(proc_dir.name)
    try:
        for fd in (proc_dir / 'fd').iterdir():
            try:
                target = os.readlink(fd).removesuffix(' (deleted)')
            except OSError:
                continue
            if target in shm_paths:
                records.add(('fd', pid, target, cmd))
    except OSError:
        pass
    try:
        for line in (proc_dir / 'maps').read_text(errors='replace').splitlines():
            target = line.split()[-1].removesuffix(' (deleted)')
            if target in shm_paths:
                records.add(('mmap', pid, target, cmd))
    except (OSError, IndexError):
        pass

try:
    lock_lines = pathlib.Path('/proc/locks').read_text().splitlines()
except OSError:
    lock_lines = []
for line in lock_lines:
    fields = line.split()
    if len(fields) < 6:
        continue
    try:
        major_text, minor_text, inode_text = fields[5].split(':')
        key = (int(major_text, 16), int(minor_text, 16), int(inode_text))
        owner = int(fields[4])
    except (ValueError, IndexError):
        continue
    path = shm_by_inode.get(key)
    if path is None:
        continue
    try:
        cmd = pathlib.Path(f'/proc/{owner}/cmdline').read_bytes().replace(
            b'\0', b' ').decode(errors='replace').strip()
    except OSError:
        cmd = ''
    records.add(('lock', owner, path, cmd))

for kind, pid, path, cmd in sorted(records):
    safe_cmd = cmd.replace('\t', ' ').replace('\n', ' ')
    print(f'{kind}\t{pid}\t{path}\t{safe_cmd}')
PY
}

navigation_nodes() {
    command -v ros2 >/dev/null 2>&1 || return 0
    timeout 8 ros2 node list --no-daemon --spin-time 2 2>/dev/null | python3 -c '
import sys
names = {
    "process_manager", "mqtt_task_receiver", "status_reporter",
    "route_planner", "navigation_manager", "map_switch_controller",
    "nav2_container", "controller_server", "smoother_server",
    "planner_server", "behavior_server", "bt_navigator",
    "waypoint_follower", "velocity_smoother", "map_server",
    "lifecycle_manager_navigation", "lifecycle_manager_localization",
    "pcl_localization", "pose_init_node", "nav2_costmap_2d_cloud",
    "lio_sam_imuPreintegration", "lio_sam_imageProjection",
    "lio_sam_featureExtraction", "lio_sam_mapOptimization",
    "ekf_gps", "navsat_transform",
}
for line in sys.stdin:
    node = line.strip()
    if node.rsplit("/", 1)[-1] in names:
        print(node)
' || true
}

write_inventory() {
    local destination="$1"
    inventory_processes | tee "$destination"
}

wait_for_no_processes() {
    local seconds="$1"
    local deadline
    deadline="$(python3 -c "import time; print(time.monotonic() + float('${seconds}'))")"
    while python3 -c "import time; raise SystemExit(0 if time.monotonic() < float('${deadline}') else 1)"; do
        if [[ -z "$(inventory_processes)" ]]; then
            return 0
        fi
        sleep 0.25
    done
    [[ -z "$(inventory_processes)" ]]
}

signal_inventory() {
    local signal_name="$1"
    local inventory_file="$2"
    python3 - "$signal_name" "$inventory_file" <<'PY'
import os
import pathlib
import signal
import sys

signal_name, inventory_path = sys.argv[1:]
signum = getattr(signal, signal_name)
for line in pathlib.Path(inventory_path).read_text().splitlines():
    fields = line.split('\t', 7)
    if len(fields) != 8:
        continue
    category, pid_text, _ppid, _pgid, expected_start, _state, _exe, cmd = fields
    pid = int(pid_text)
    try:
        stat_tail = pathlib.Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
        continue
    if stat_tail[19] != expected_start or stat_tail[0] == 'Z':
        continue
    try:
        os.kill(pid, signum)
        print(f'{signal_name} -> PID {pid} [{category}] {cmd}')
    except ProcessLookupError:
        pass
    except PermissionError as exc:
        print(f'无法向PID {pid}发送{signal_name}: {exc}', file=sys.stderr)
PY
}

BEFORE_FILE="${REPORT_DIR}/processes_before.tsv"
NODES_BEFORE_FILE="${REPORT_DIR}/ros_nodes_before.txt"
echo
echo "关闭前导航进程："
write_inventory "$BEFORE_FILE" || true
navigation_nodes | tee "$NODES_BEFORE_FILE" || true
DAEMONS_BEFORE_FILE="${REPORT_DIR}/ros2_cli_daemons_before.tsv"
echo "关闭前ROS 2 CLI守护进程："
inventory_ros_cli_daemons | tee "$DAEMONS_BEFORE_FILE" || true
FAST_DDS_BEFORE_FILE="${REPORT_DIR}/fastdds_helpers_before.tsv"
echo "关闭前独立Fast DDS discovery进程："
inventory_fastdds_helpers | tee "$FAST_DDS_BEFORE_FILE" || true
FAST_DDS_USAGE_BEFORE_FILE="${REPORT_DIR}/fastdds_usage_before.tsv"
echo "关闭前Fast DDS FD/mmap/内核锁："
inventory_fastdds_usage | tee "$FAST_DDS_USAGE_BEFORE_FILE" || true

if [[ ${CHECK_ONLY} -eq 0 ]]; then
    if command -v ros2 >/dev/null 2>&1; then
        ROS_SERVICES_FILE="${REPORT_DIR}/ros_services_before.txt"
        timeout 5 ros2 service list >"$ROS_SERVICES_FILE" 2>/dev/null || true
        if grep -Fxq /process_manager/shutdown_process "$ROS_SERVICES_FILE"; then
            echo
            echo "请求管理器停止活动任务……"
            timeout 8 ros2 topic pub --once --qos-durability volatile \
                /process_manager/fault std_msgs/msg/String \
                "{data: 'stop_navigation_script: operator_stop'}" || \
                echo "未确认故障消息送达；继续执行服务和进程级收尾。"

            sleep 2
            for process_name in navigation2 liosam nav2_init_pose re_localization; do
                echo "请求关闭子栈：${process_name}"
                timeout "${SERVICE_TIMEOUT}" ros2 service call \
                    /process_manager/shutdown_process \
                    multi_map_navigation_msgs/srv/ShutdownProcess \
                    "{process_name: '${process_name}'}" || \
                    echo "${process_name}服务关闭未确认；稍后进行进程级收尾。"
            done

            if grep -Fxq /process_manager/get_status "$ROS_SERVICES_FILE"; then
                echo "子栈服务状态："
                timeout "${SERVICE_TIMEOUT}" ros2 service call \
                    /process_manager/get_status \
                    multi_map_navigation_msgs/srv/GetProcessStatus "{}" || true
            fi
        else
            echo "未发现process_manager关闭服务；跳过ROS请求，直接进行进程级收尾。"
        fi
    else
        echo "未找到ros2命令；跳过ROS消息和服务，进行进程级收尾。"
    fi

    # Give ros2 launch the first chance to perform its own shutdown handlers.
    MANAGEMENT_FILE="${REPORT_DIR}/management_launch.tsv"
    inventory_processes | awk -F '\t' '$1 == "management-launch" {print}' >"$MANAGEMENT_FILE"
    if [[ -s "$MANAGEMENT_FILE" ]]; then
        echo
        echo "向导航管理launch发送SIGINT……"
        signal_inventory SIGINT "$MANAGEMENT_FILE"
        wait_for_no_processes "$GRACE_SECONDS" || true
    fi

    RESIDUAL_FILE="${REPORT_DIR}/residual_before_sigint.tsv"
    inventory_processes >"$RESIDUAL_FILE"
    if [[ -s "$RESIDUAL_FILE" ]]; then
        echo
        echo "向明确归属导航栈的残留发送SIGINT……"
        signal_inventory SIGINT "$RESIDUAL_FILE"
        wait_for_no_processes "$GRACE_SECONDS" || true
    fi

    RESIDUAL_FILE="${REPORT_DIR}/residual_before_sigterm.tsv"
    inventory_processes >"$RESIDUAL_FILE"
    if [[ -s "$RESIDUAL_FILE" ]]; then
        echo
        echo "SIGINT后仍有残留，发送SIGTERM……"
        signal_inventory SIGTERM "$RESIDUAL_FILE"
        wait_for_no_processes "$GRACE_SECONDS" || true
    fi

    RESIDUAL_FILE="${REPORT_DIR}/residual_before_sigkill.tsv"
    inventory_processes >"$RESIDUAL_FILE"
    if [[ -s "$RESIDUAL_FILE" && ${FORCE_KILL} -eq 1 ]]; then
        echo
        echo "SIGTERM后仍有明确的导航残留，发送SIGKILL……"
        signal_inventory SIGKILL "$RESIDUAL_FILE"
        wait_for_no_processes 3 || true
    fi

    if command -v ros2 >/dev/null 2>&1; then
        echo
        echo "关闭ROS 2 CLI后台守护进程……"
        timeout 5 ros2 daemon stop || true
        sleep 0.5
        DAEMON_RESIDUAL_FILE="${REPORT_DIR}/ros2_cli_daemons_before_sigterm.tsv"
        inventory_ros_cli_daemons >"$DAEMON_RESIDUAL_FILE"
        if [[ -s "$DAEMON_RESIDUAL_FILE" ]]; then
            signal_inventory SIGTERM "$DAEMON_RESIDUAL_FILE"
            sleep 1
            inventory_ros_cli_daemons >"$DAEMON_RESIDUAL_FILE"
        fi
        if [[ -s "$DAEMON_RESIDUAL_FILE" && ${FORCE_KILL} -eq 1 ]]; then
            signal_inventory SIGKILL "$DAEMON_RESIDUAL_FILE"
            sleep 0.5
        fi
    fi
fi

AFTER_FILE="${REPORT_DIR}/processes_after.tsv"
NODES_AFTER_FILE="${REPORT_DIR}/ros_nodes_after.txt"
echo
echo "最终导航进程检查："
write_inventory "$AFTER_FILE" || true

echo "最终导航ROS节点检查："
navigation_nodes | tee "$NODES_AFTER_FILE" || true

DAEMONS_AFTER_FILE="${REPORT_DIR}/ros2_cli_daemons_after.tsv"
echo "最终ROS 2 CLI守护进程检查："
inventory_ros_cli_daemons | tee "$DAEMONS_AFTER_FILE" || true

FAST_DDS_AFTER_FILE="${REPORT_DIR}/fastdds_helpers_after.tsv"
echo "最终独立Fast DDS discovery进程检查："
inventory_fastdds_helpers | tee "$FAST_DDS_AFTER_FILE" || true

FAST_DDS_SHM_FILE="${REPORT_DIR}/fastdds_shm_after.txt"
inventory_fastdds_shm >"$FAST_DDS_SHM_FILE"
FAST_DDS_USAGE_FILE="${REPORT_DIR}/fastdds_usage_after.tsv"
echo "最终Fast DDS FD/mmap/内核锁检查："
inventory_fastdds_usage | tee "$FAST_DDS_USAGE_FILE" || true

PROCESS_COUNT="$(wc -l <"$AFTER_FILE")"
NODE_COUNT="$(wc -l <"$NODES_AFTER_FILE")"
DAEMON_COUNT="$(wc -l <"$DAEMONS_AFTER_FILE")"
FAST_DDS_COUNT="$(wc -l <"$FAST_DDS_AFTER_FILE")"
FAST_DDS_SHM_COUNT="$(wc -l <"$FAST_DDS_SHM_FILE")"
FAST_DDS_USAGE_COUNT="$(wc -l <"$FAST_DDS_USAGE_FILE")"
FAST_DDS_LOCK_COUNT="$(awk -F '\t' '$1 == "lock" {count++} END {print count + 0}' "$FAST_DDS_USAGE_FILE")"
SUMMARY_FILE="${REPORT_DIR}/summary.txt"
{
    echo "finished_at=$(date -Ins)"
    echo "mode=$([[ ${CHECK_ONLY} -eq 1 ]] && echo check-only || echo stop-and-check)"
    echo "navigation_process_residuals=${PROCESS_COUNT}"
    echo "navigation_node_residuals=${NODE_COUNT}"
    echo "ros2_cli_daemon_residuals=${DAEMON_COUNT}"
    echo "fastdds_discovery_processes=${FAST_DDS_COUNT}"
    echo "fastdds_shm_objects_observed=${FAST_DDS_SHM_COUNT}"
    echo "fastdds_visible_fd_mmap_lock_records=${FAST_DDS_USAGE_COUNT}"
    echo "fastdds_matching_kernel_locks=${FAST_DDS_LOCK_COUNT}"
    echo "sensor_drivers_targeted=false"
} | tee "$SUMMARY_FILE"

if [[ "$PROCESS_COUNT" -eq 0 && "$NODE_COUNT" -eq 0 &&
        "$DAEMON_COUNT" -eq 0 && "$FAST_DDS_COUNT" -eq 0 ]]; then
    echo "结论：未发现导航栈、ROS 2 CLI守护进程或独立Fast DDS discovery进程残留。"
    echo "传感器和底盘驱动未纳入停止范围；Fast DDS共享内存清单已记录但未删除。"
    exit 0
fi

echo "结论：仍发现收尾检查项。请查看："
echo "  ${AFTER_FILE}"
echo "  ${NODES_AFTER_FILE}"
echo "  ${DAEMONS_AFTER_FILE}"
echo "  ${FAST_DDS_AFTER_FILE}"
echo "  ${FAST_DDS_USAGE_FILE}"
if [[ "$FAST_DDS_COUNT" -ne 0 ]]; then
    echo "独立Fast DDS discovery进程未自动停止；请先确认传感器不依赖它。"
fi
if [[ ${CHECK_ONLY} -eq 1 ]]; then
    echo "执行不带--check-only的命令可进行关闭。"
elif [[ ${FORCE_KILL} -eq 0 ]]; then
    echo "本次使用了--no-force-kill；确认身份后可重新执行默认关闭。"
fi
exit 2
