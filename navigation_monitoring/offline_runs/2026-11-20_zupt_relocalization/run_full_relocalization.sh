#!/usr/bin/env bash
set -euo pipefail

root=/home/akun/workspace/Car_jetson
run_root="$root/navigation_monitoring/offline_runs/2026-11-20_zupt_relocalization/runtime/relocalization_full_map2_1109"
bag="$root/navigation_monitoring/runs/2026-11-20_10-25-54_manual_publish_monitor/navigation_core_bag_segment_2"
map="$root/multi_map_navigation/src/map/map2/map2_raw.pcd"
params="$root/utils/src/re_localization/param/localization_real.yaml"
csv="$root/multi_map_navigation/src/map/map2/map2.csv"
map_yaml="$root/multi_map_navigation/src/map/map2/map2_clean.yaml"

if [[ -e "$run_root" ]]; then
  echo "refusing to overwrite existing run: $run_root" >&2
  exit 3
fi
mkdir -p "$run_root/logs"
set +u
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
set -u
export ROS_DOMAIN_ID=95
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$run_root/ros_logs"
mkdir -p "$ROS_LOG_DIR"

session_id=offline-full-relocalization-map2-1109-20261120
session_start_ns=1795144183000000000
start_offset=1420

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
  --topics /rslidar_Pointcloud2 /sensing/gnss/pose_with_covariance \
  >"$run_root/logs/play.log" 2>&1 &
player_pgid="$!"
groups+=("$player_pgid")

# Start the replay before the client so the first GPS/cloud sample is already
# available when nav2_init_pose invokes the service.  The explicit type keeps
# ros2 topic echo from waiting for type discovery and aborting the harness.
sleep 8
setsid ros2 topic echo --once /localization_result \
  multi_map_navigation_msgs/msg/LocalizationResult \
  >"$run_root/localization_result.yaml" 2>"$run_root/logs/result_echo.log" &
result_pgid="$!"
groups+=("$result_pgid")

setsid ros2 run nav2_init_pose nav2_init_pose \
  --ros-args -p use_sim_time:=true \
  -p map_csv:="$csv" -p map_yaml:="$map_yaml" -p map_name:=map2 \
  -p localization_session_id:="$session_id" -p session_start_ns:="$session_start_ns" \
  >"$run_root/logs/nav2_init_pose.log" 2>&1 &
init_pgid="$!"
groups+=("$init_pgid")

for _ in $(seq 1 90); do
  if rg -q '^success:' "$run_root/localization_result.yaml" 2>/dev/null; then break; fi
  if ! kill -0 "$init_pgid" 2>/dev/null; then break; fi
  sleep 1
done
if rg -q '^success:' "$run_root/localization_result.yaml" 2>/dev/null \
    && kill -0 "$init_pgid" 2>/dev/null; then
  stop_group "$init_pgid"
fi
set +e
wait "$init_pgid"
init_rc="$?"
set -e
echo "$init_rc" >"$run_root/init_exit_code.txt"
sleep 2
cat "$run_root/localization_result.yaml" 2>/dev/null || true
cat "$run_root/logs/nav2_init_pose.log"

