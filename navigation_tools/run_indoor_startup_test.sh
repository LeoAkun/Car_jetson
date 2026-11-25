#!/usr/bin/env bash
# Live IMU/LiDAR startup test, with no task source or chassis command bridge.
set -eo pipefail
TEST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
for test_ws in real_ws utils slam nav2; do
    source "${TEST_ROOT}/${test_ws}/install/setup.bash"
done
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1
exec /usr/bin/python3 "${TEST_ROOT}/navigation_tools/indoor_startup_test.py" "$@"
