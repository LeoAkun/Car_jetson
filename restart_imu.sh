#!/usr/bin/env bash
set -Ee -o pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="/home/akun/workspace/Car_jetson/real_ws/install/setup.bash"
IMU_NODE="/home/akun/workspace/Car_jetson/real_ws/install/fdilink_ahrs/lib/fdilink_ahrs/ahrs_driver_node"
IMU_DEVICE="/dev/wheeltec_FDI_IMU_GNSS"
LOG_DIR="${HOME}/.local/state/car_jetson"
LOG_FILE="${LOG_DIR}/imu_driver.log"
PID_FILE="${LOG_DIR}/imu_driver.pid"
LOCK_FILE="/tmp/car_jetson_restart_imu.lock"

usage() {
    echo "Usage: $0 [--force]"
    echo "  --force  Restart even when /imu is currently publishing data."
}

force_restart=false
case "${1:-}" in
    "") ;;
    --force) force_restart=true ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

for required_file in "${ROS_SETUP}" "${WS_SETUP}" "${IMU_NODE}"; do
    if [[ ! -e "${required_file}" ]]; then
        echo "Required file not found: ${required_file}" >&2
        exit 1
    fi
done

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_SETUP}"

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "Another IMU restart is already running." >&2
    exit 1
fi

imu_has_data() {
    local timeout_seconds="$1"
    timeout "${timeout_seconds}" ros2 topic echo --no-daemon --spin-time 1 \
        /imu sensor_msgs/msg/Imu \
        --once --field header.stamp --no-lost-messages \
        >/dev/null 2>&1
}

get_imu_pids() {
    pgrep -f -- "^${IMU_NODE}( |$)" || true
}

if [[ "${force_restart}" == false ]] && imu_has_data 3; then
    echo "IMU data is healthy; restart skipped. Use --force to override."
    exit 0
fi

if [[ ! -e "${IMU_DEVICE}" ]]; then
    echo "IMU device not found: ${IMU_DEVICE}" >&2
    echo "Reconnect the IMU cable, then run this script again." >&2
    exit 1
fi

if [[ ! -r "${IMU_DEVICE}" || ! -w "${IMU_DEVICE}" ]]; then
    echo "No read/write permission for ${IMU_DEVICE}." >&2
    echo "Run: sudo chmod a+rw ${IMU_DEVICE}" >&2
    exit 1
fi

mapfile -t old_pids < <(get_imu_pids)
if ((${#old_pids[@]} > 0)); then
    echo "Stopping old IMU process(es): ${old_pids[*]}"
    kill -INT "${old_pids[@]}" 2>/dev/null || true

    for _ in {1..50}; do
        remaining=()
        for pid in "${old_pids[@]}"; do
            if kill -0 "${pid}" 2>/dev/null; then
                remaining+=("${pid}")
            fi
        done
        ((${#remaining[@]} == 0)) && break
        sleep 0.1
    done

    if ((${#remaining[@]} > 0)); then
        echo "IMU did not stop after SIGINT; sending SIGTERM: ${remaining[*]}"
        kill -TERM "${remaining[@]}" 2>/dev/null || true
        sleep 1
    fi

    stubborn=()
    for pid in "${remaining[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            stubborn+=("${pid}")
        fi
    done
    if ((${#stubborn[@]} > 0)); then
        echo "IMU did not stop after SIGTERM; sending SIGKILL: ${stubborn[*]}"
        kill -KILL "${stubborn[@]}" 2>/dev/null || true
        sleep 0.5
    fi
fi

mapfile -t remaining_pids < <(get_imu_pids)
if ((${#remaining_pids[@]} > 0)); then
    echo "Unable to stop old IMU process(es): ${remaining_pids[*]}" >&2
    exit 1
fi

mkdir -p "${LOG_DIR}"
touch "${LOG_FILE}"

echo "Starting one IMU driver..."
nohup setsid "${IMU_NODE}" --ros-args \
    -p if_debug_:=false \
    -p serial_port_:="${IMU_DEVICE}" \
    -p serial_baud_:=921600 \
    -p imu_topic:=/imu \
    -p imu_frame_id_:=imu_link \
    -p device_type_:=1 \
    >>"${LOG_FILE}" 2>&1 </dev/null 9>&- &
new_pid=$!
echo "${new_pid}" >"${PID_FILE}"

for _ in {1..8}; do
    if ! kill -0 "${new_pid}" 2>/dev/null; then
        echo "IMU driver exited during startup. Last log lines:" >&2
        tail -n 20 "${LOG_FILE}" >&2
        exit 1
    fi

    if imu_has_data 2; then
        mapfile -t current_pids < <(get_imu_pids)
        if ((${#current_pids[@]} != 1)); then
            echo "Expected one IMU process, found ${#current_pids[@]}: ${current_pids[*]}" >&2
            exit 1
        fi

        echo "IMU restarted successfully (PID ${current_pids[0]})."
        echo "Log: ${LOG_FILE}"
        exit 0
    fi
done

echo "IMU process is running, but /imu has no data." >&2
echo "Check the cable and log: ${LOG_FILE}" >&2
exit 1
