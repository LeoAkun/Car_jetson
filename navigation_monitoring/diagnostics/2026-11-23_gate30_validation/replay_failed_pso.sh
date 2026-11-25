#!/usr/bin/env bash
set -euo pipefail

root=/home/akun/workspace/Car_jetson
run_root="$root/navigation_monitoring/diagnostics/2026-11-23_gate30_validation/pso_trial_2"
bag="$root/navigation_monitoring/runs/2026-11-23_11-41-36_manual_publish_monitor/navigation_core_bag_segment_2"
map="$root/multi_map_navigation/src/map/map2/map2_raw.pcd"
params="$root/utils/src/re_localization/param/localization_real.yaml"
csv="$root/multi_map_navigation/src/map/map2/map2.csv"
map_yaml="$root/multi_map_navigation/src/map/map2/map2_clean.yaml"

if [[ -e "$run_root" ]]; then
  echo "refusing to overwrite existing run: $run_root" >&2
  exit 3
fi
mkdir -p "$run_root/logs" "$run_root/ros_logs"
source "$root/navigation_tools/ros2_navigation_env.bash"
export ROS_DOMAIN_ID=197
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$run_root/ros_logs"
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE

session_id=offline-nov23-failed-pso-trial-2
session_start_ns=1795406971000000000
start_offset=245

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
groups+=("$!")

for _ in $(seq 1 90); do
  if ros2 service list 2>/dev/null | rg -q '^/re_localization$'; then break; fi
  sleep 1
done
if ! ros2 service list 2>/dev/null | rg -q '^/re_localization$'; then
  echo 're_localization service unavailable' >&2
  exit 4
fi

setsid ros2 bag play "$bag" --clock 100 --rate 1.0 --start-offset "$start_offset" \
  --disable-keyboard-controls --read-ahead-queue-size 128 \
  --topics /rslidar_Pointcloud2 /sensing/gnss/pose_with_covariance \
  >"$run_root/logs/play.log" 2>&1 &
groups+=("$!")
sleep 8

setsid ros2 topic echo --once /localization_result \
  multi_map_navigation_msgs/msg/LocalizationResult \
  >"$run_root/localization_result.yaml" 2>"$run_root/logs/result_echo.log" &
groups+=("$!")

set +e
timeout --signal=INT --kill-after=10s 145s ros2 run nav2_init_pose nav2_init_pose \
  --ros-args -p use_sim_time:=true \
  -p map_csv:="$csv" -p map_yaml:="$map_yaml" -p map_name:=map2 \
  -p localization_session_id:="$session_id" \
  -p session_start_ns:="$session_start_ns" \
  >"$run_root/logs/nav2_init_pose.log" 2>&1
init_rc="$?"
set -e
printf '%s\n' "$init_rc" >"$run_root/init_exit_code.txt"
sleep 2

if [[ "$init_rc" -eq 0 ]]; then
  echo 'invalid PSO replay unexpectedly succeeded' >&2
  exit 5
fi
if rg -q '^success: true' "$run_root/localization_result.yaml"; then
  echo 'invalid PSO replay published a successful localization result' >&2
  exit 6
fi
if ! rg -q 'PSO搜索无有效结果|localization exceeded 120 second deadline|PSO returned no valid pose' \
    "$run_root/logs/nav2_init_pose.log"; then
  echo 'expected bounded PSO failure was not observed' >&2
  exit 7
fi
echo "PSO rejection verified, nav2_init_pose exit=$init_rc"
