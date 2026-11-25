#!/usr/bin/env bash
set -euo pipefail

project=/home/akun/workspace/Car_jetson
change_dir="$project/navigation_monitoring/config_changes/2026-11-10_global_planner_geometry_fix"

cp "$change_dir/before/real_ws/src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml" \
  "$project/real_ws/src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml"
cp "$change_dir/before/nav2/src/nav2/config/nav2_real_params.yaml" \
  "$project/nav2/src/nav2/config/nav2_real_params.yaml"
cp "$change_dir/before/real_ws/src/driver/car_control/cmdvel_2_ecu/cmdvel_2_ecu/convert.py" \
  "$project/real_ws/src/driver/car_control/cmdvel_2_ecu/cmdvel_2_ecu/convert.py"
cp "$change_dir/before/real_ws/src/driver/car_control/cmdvel_2_ecu/config/config.yaml" \
  "$project/real_ws/src/driver/car_control/cmdvel_2_ecu/config/config.yaml"
cp "$change_dir/before/real_ws/src/driver/car_control/cmdvel_2_ecu/config/docking_config.yaml" \
  "$project/real_ws/src/driver/car_control/cmdvel_2_ecu/config/docking_config.yaml"
cp "$change_dir/before/real_ws/src/driver/car_control/cmdvel_2_ecu/test/test_docking_conversion.py" \
  "$project/real_ws/src/driver/car_control/cmdvel_2_ecu/test/test_docking_conversion.py"
cp "$change_dir/before/multi_map_navigation/src/multi_map_navigation/test/test_navigation_recovery.py" \
  "$project/multi_map_navigation/src/multi_map_navigation/test/test_navigation_recovery.py"

source /opt/ros/humble/setup.bash
source "$project/real_ws/install/setup.bash"
cd "$project/real_ws"
colcon build --symlink-install --packages-select ros2_laser_scan_merger cmdvel_2_ecu

cd "$project/nav2"
colcon build --symlink-install --packages-select nav2

