#!/usr/bin/env bash
set -euo pipefail

root=/home/akun/workspace/Car_jetson
run_root="$root/navigation_monitoring/offline_runs/2026-11-20_zupt_relocalization/runtime/relocalization_map2_1109"
bag="$root/navigation_monitoring/runs/2026-11-20_10-25-54_manual_publish_monitor/navigation_core_bag_segment_2"
map="$root/multi_map_navigation/src/map/map2/map2_raw.pcd"
params="$root/utils/src/re_localization/param/localization_real.yaml"
probe="$root/navigation_monitoring/offline_runs/2026-11-20_zupt_relocalization/call_relocalization.py"

if [[ -e "$run_root" ]]; then
  echo "refusing to overwrite existing run: $run_root" >&2
  exit 3
fi
mkdir -p "$run_root/logs"
set +u
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
set -u
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$run_root/ros_logs"
mkdir -p "$ROS_LOG_DIR"

session_id=offline-relocalization-map2-1109-20261120
session_start_ns=1795144183000000000
start_offset=1422
initial_x=-202.12
initial_y=84.85
initial_yaw_deg=90.0

declare -a groups=()
stop_group() {
  local pgid="$1"
  if kill -0 "$pgid" 2>/dev/null; then
    kill -INT -- "-$pgid" 2>/dev/null || true
  fi
}
cleanup() {
  set +e
  for ((i=${#groups[@]}-1; i>=0; i--)); do stop_group "${groups[$i]}"; done
  sleep 2
  for ((i=${#groups[@]}-1; i>=0; i--)); do
    if kill -0 "${groups[$i]}" 2>/dev/null; then
      kill -TERM -- "-${groups[$i]}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

setsid ros2 launch re_localization run.real_launch.py \
  map_path:="$map" localization_param_dir:="$params" \
  localization_session_id:="$session_id" map_id:=map2 \
  session_start_ns:="$session_start_ns" \
  >"$run_root/logs/re_localization.log" 2>&1 &
server_pgid="$!"
groups+=("$server_pgid")

for _ in $(seq 1 60); do
  if ros2 service list 2>/dev/null | rg -q '^/re_localization$'; then break; fi
  sleep 1
done
if ! ros2 service list 2>/dev/null | rg -q '^/re_localization$'; then
  echo 're_localization service did not become available' >&2
  exit 4
fi

setsid ros2 bag play "$bag" --clock 100 --rate 0.5 --start-offset "$start_offset" \
  --topics /rslidar_Pointcloud2 /sensing/gnss/pose_with_covariance \
  >"$run_root/logs/play.log" 2>&1 &
player_pgid="$!"
groups+=("$player_pgid")
sleep 5

set +e
python3 "$probe" --session-id "$session_id" --map-id map2 \
  --session-start-ns "$session_start_ns" --observation-id 1 \
  --min-cloud-ns "$session_start_ns" --x "$initial_x" --y "$initial_y" \
  --yaw-deg "$initial_yaw_deg" --refresh-cloud \
  >"$run_root/observation_1.json"
rc1=$?
set -e
if [[ "$rc1" -ne 0 ]]; then exit 5; fi

cloud_1="$(jq -r '.cloud_stamp_ns' "$run_root/observation_1.json")"
if [[ ! "$cloud_1" =~ ^[0-9]+$ ]]; then exit 6; fi

set +e
python3 "$probe" --session-id "$session_id" --map-id map2 \
  --session-start-ns "$session_start_ns" --observation-id 2 \
  --min-cloud-ns "$cloud_1" --x "$initial_x" --y "$initial_y" \
  --yaw-deg "$initial_yaw_deg" --refresh-cloud \
  >"$run_root/observation_2.json"
rc2=$?
set -e
if [[ "$rc2" -ne 0 ]]; then exit 7; fi

echo "completed obs1=$(jq -c . "$run_root/observation_1.json")"
echo "completed obs2=$(jq -c . "$run_root/observation_2.json")"

