#!/usr/bin/env bash
# Offline software acceptance only. Never starts ECU/CAN, MQTT or sensor drivers.
set -e
SIM_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /home/akun/car-host-step9.MgehLL/environment.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="${SIM_DIR}/localhost-dds.xml"
export ROS_LOG_DIR="${SIM_DIR}/ros-log"
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
unset ROBOT_HARDWARE_COMMISSIONED
cd "${SIM_DIR}"
exec python3 -m pytest -q -s \
  test_navigation.py test_task_navigation.py test_localization.py test_slam_replay.py \
  test_route_recovery_integration.py \
  /home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/test/test_map_switch_controller_real_deps.py \
  /home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/test/test_navigation_recovery.py \
  /home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/test/test_process_health.py \
  --junitxml="${SIM_DIR}/all-results.xml"
