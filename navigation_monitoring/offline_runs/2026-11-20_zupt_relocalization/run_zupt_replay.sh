#!/usr/bin/env bash
set -euo pipefail

# Software-only, localhost-isolated replay.  This script never launches vehicle
# drivers or publishes control, fault, or navigation-task topics.

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <zupt:on|off> <bag-seconds>" >&2
  exit 2
fi

variant="$1"
bag_seconds="$2"
case "$variant" in
  on) zupt_value=true ;;
  off) zupt_value=false ;;
  *) echo "variant must be on or off" >&2; exit 2 ;;
esac
[[ "$bag_seconds" =~ ^[0-9]+$ ]] || { echo "bag-seconds must be an integer" >&2; exit 2; }

root=/home/akun/workspace/Car_jetson
run_root="$root/navigation_monitoring/offline_runs/2026-11-20_zupt_relocalization/runtime/zupt_${variant}_rate05_${bag_seconds}s"
input_bag="$root/navigation_monitoring/runs/2026-11-20_10-25-54_manual_publish_monitor/navigation_core_bag_segment_3"
params="$root/slam/src/LIO-SAM/config/params_rslidar.yaml"
output_bag="$run_root/output_bag"

if [[ -e "$run_root" ]]; then
  echo "refusing to overwrite existing run: $run_root" >&2
  exit 3
fi
mkdir -p "$run_root/logs"

set +u
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
set -u
export ROS_DOMAIN_ID=98
export ROS_LOCALHOST_ONLY=1
export LIO_SAM_INSTANCE_ID="offline-zupt-${variant}-20261120"
export LIO_SAM_GENERATION=1
export ROS_LOG_DIR="$run_root/ros_logs"
mkdir -p "$ROS_LOG_DIR"

lio_prefix="$(ros2 pkg prefix lio_sam)"
declare -a child_pgids=()

stop_group() {
  local pgid="$1"
  if kill -0 "$pgid" 2>/dev/null; then
    kill -INT -- "-$pgid" 2>/dev/null || true
  fi
}

cleanup() {
  set +e
  for ((i=${#child_pgids[@]}-1; i>=0; i--)); do
    stop_group "${child_pgids[$i]}"
  done
  sleep 2
  for ((i=${#child_pgids[@]}-1; i>=0; i--)); do
    if kill -0 "${child_pgids[$i]}" 2>/dev/null; then
      kill -TERM -- "-${child_pgids[$i]}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_lio() {
  local executable="$1"
  shift
  setsid "$lio_prefix/lib/lio_sam/$executable" \
    --ros-args --params-file "$params" -p use_sim_time:=true -p savePCD:=false "$@" \
    >"$run_root/logs/${executable}.log" 2>&1 &
  child_pgids+=("$!")
}

start_lio lio_sam_imuPreintegration -p "useVehicleZeroVelocityUpdate:=$zupt_value"
start_lio lio_sam_imageProjection
start_lio lio_sam_featureExtraction
start_lio lio_sam_mapOptimization

sleep 4
for pgid in "${child_pgids[@]}"; do
  kill -0 "$pgid"
done

record_topics=(
  /odom
  /odom_incremental
  /lio_sam/mapping/odometry
  /lio_sam/mapping/odometry_incremental
  /lio_sam/epoch_odometry
  /lio_sam/correction_event
  /lio_sam/mapping/correction_event
  /lio_sam/map_odom_correction
  /lio_sam/localization_fault
  /lio_sam/imu_preintegration_ready
  /lio_sam/imu_preintegration_readiness
)
setsid ros2 bag record -o "$output_bag" "${record_topics[@]}" \
  >"$run_root/logs/record.log" 2>&1 &
recorder_pgid="$!"
child_pgids+=("$recorder_pgid")
sleep 2

# At 0.5x, allow exactly the requested input duration plus ten wall seconds
# for player startup. timeout is only a bounding watchdog; SIGINT flushes cleanly.
wall_limit=$((bag_seconds * 2 + 10))
set +e
setsid timeout --signal=INT --kill-after=8 "$wall_limit" \
  ros2 bag play "$input_bag" --clock 100 --rate 0.5 \
    --topics /imu /vehicle_status /rslidar_Pointcloud2 /tf_static \
  >"$run_root/logs/play.log" 2>&1 &
player_pgid="$!"
child_pgids+=("$player_pgid")
wait "$player_pgid"
player_rc="$?"
set -e
echo "$player_rc" >"$run_root/player_exit_code.txt"

stop_group "$recorder_pgid"
sleep 3

for node_index in 0 1 2 3; do
  pgid="${child_pgids[$node_index]}"
  if ! kill -0 "$pgid" 2>/dev/null; then
    echo "LIO child exited early: $pgid" >&2
    exit 4
  fi
done

echo "completed variant=$variant rate=0.5 bag_seconds=$bag_seconds player_rc=$player_rc"

