#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/real_ws/install/setup.bash"

set -u

exec ros2 run lock_controller open_door_recover.py "$@"
