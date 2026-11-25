#!/usr/bin/env bash
set -euo pipefail

root=/home/akun/workspace/Car_jetson
run_root="$root/navigation_monitoring/offline_runs/2026-11-20_zupt_relocalization/runtime/relocalization_weak_20261118"
bag="$root/navigation_monitoring/runs/2026-11-18_11-25-25_manual_publish_monitor/navigation_core_bag_segment_3"
map="$root/multi_map_navigation/src/map/map2/map2_raw.pcd"
params="$root/utils/src/re_localization/param/localization_real.yaml"
validator="$root/navigation_monitoring/offline_runs/2026-11-20_zupt_relocalization/validate_weak_candidate.py"

if [[ -e "$run_root" ]]; then
  echo "refusing to overwrite existing run: $run_root" >&2
  exit 3
fi
mkdir -p "$run_root/logs"
set +u
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
set -u
export ROS_DOMAIN_ID=96
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$run_root/ros_logs"
mkdir -p "$ROS_LOG_DIR"

session_id=offline-weak-map2-20261118
session_start_ns=1794974769500000000
start_offset=1120

declare -a groups=()
stop_group() {
  local pgid="$1"
  if kill -0 "$pgid" 2>/dev/null; then kill -INT -- "-$pgid" 2>/dev/null || true; fi
}
cleanup() {
  set +e
  for ((i=${#groups[@]}-1; i>=0; i--)); do stop_group "${groups[$i]}"; done
  sleep 2
  for ((i=${#groups[@]}-1; i>=0; i--)); do
    if kill -0 "${groups[$i]}" 2>/dev/null; then kill -TERM -- "-${groups[$i]}" 2>/dev/null || true; fi
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
if ! ros2 service list 2>/dev/null | rg -q '^/re_localization$'; then exit 4; fi

setsid ros2 bag play "$bag" --clock 100 --rate 0.5 --start-offset "$start_offset" \
  --topics /rslidar_Pointcloud2 \
  >"$run_root/logs/play.log" 2>&1 &
player_pgid="$!"
groups+=("$player_pgid")
sleep 12

python3 "$validator" \
  --x -132.01 --y 167.89 --yaw-deg 45 \
  --prior-x -131.17 --prior-y 168.39 \
  --ros-args -p use_sim_time:=true \
  -p map_csv:="$root/multi_map_navigation/src/map/map2/map2.csv" \
  -p map_yaml:="$root/multi_map_navigation/src/map/map2/map2_clean.yaml" \
  -p map_name:=map2 -p localization_session_id:="$session_id" \
  -p session_start_ns:="$session_start_ns" \
  >"$run_root/result.json" 2>"$run_root/logs/validator.stderr.log"

cat "$run_root/result.json"

