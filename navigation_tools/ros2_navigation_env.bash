#!/usr/bin/env bash
# Complete ROS 2 environment for the vehicle. This file is meant to be sourced.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf '请使用 source %s\n' "${BASH_SOURCE[0]}" >&2
    exit 2
fi

_car_ros_env_root="/home/akun/workspace/Car_jetson"
_car_ros_env_script="${_car_ros_env_root}/navigation_tools/ros2_navigation_env.bash"

# A new terminal starts with a clean environment. In an existing terminal,
# avoid repeatedly appending the same prefixes when .bashrc is sourced again.
if [[ "${CAR_JETSON_ROS_ENV_LOADED_FROM:-}" == "${_car_ros_env_script}" ]]; then
    unset _car_ros_env_root _car_ros_env_script
    return 0
fi

_car_ros_env_had_nounset=0
if [[ $- == *u* ]]; then
    _car_ros_env_had_nounset=1
    set +u
fi

_car_ros_env_base="/opt/ros/humble/setup.bash"
if [[ ! -r "${_car_ros_env_base}" ]]; then
    printf 'ROS 2 基础环境不存在或不可读: %s\n' "${_car_ros_env_base}" >&2
    if (( _car_ros_env_had_nounset )); then
        set -u
    fi
    unset _car_ros_env_base _car_ros_env_had_nounset \
        _car_ros_env_root _car_ros_env_script
    return 1
fi

# Use local_setup.bash deliberately. Every overlay is listed here explicitly,
# so the result does not depend on which underlays happened to be active when
# a workspace's generated setup.bash was last written by colcon.
# shellcheck disable=SC1091
source "${_car_ros_env_base}"

_car_ros_env_missing=()
_car_ros_env_overlays=(
    "real_ws/install/local_setup.bash"
    "utils/install/local_setup.bash"
    "slam/install/local_setup.bash"
    "nav2/install/local_setup.bash"
    "multi_map_navigation/install/local_setup.bash"
    "install/local_setup.bash"
)

for _car_ros_env_relative in "${_car_ros_env_overlays[@]}"; do
    _car_ros_env_setup="${_car_ros_env_root}/${_car_ros_env_relative}"
    if [[ -r "${_car_ros_env_setup}" ]]; then
        # shellcheck disable=SC1090
        source "${_car_ros_env_setup}"
    else
        _car_ros_env_missing+=("${_car_ros_env_setup}")
    fi
done

export CAR_JETSON_ROS_ENV_LOADED_FROM="${_car_ros_env_script}"

if ((${#_car_ros_env_missing[@]} > 0)); then
    printf '警告: 以下车辆 ROS 2 工作空间环境不存在，本终端环境可能不完整:\n' >&2
    printf '  %s\n' "${_car_ros_env_missing[@]}" >&2
fi

# Run this after a rebuild or whenever a terminal's environment is in doubt.
car_ros_env_check() {
    local failed=0
    local package_name package_prefix

    if ! command -v ros2 >/dev/null 2>&1; then
        printf 'FAIL: ros2 命令不可用\n' >&2
        return 1
    fi

    for package_name in \
        launch_real \
        re_localization \
        lio_sam \
        nav2_simple_commander \
        multi_map_navigation \
        multi_map_navigation_msgs; do
        if package_prefix="$(ros2 pkg prefix "${package_name}" 2>/dev/null)"; then
            printf 'OK: %-28s %s\n' "${package_name}" "${package_prefix}"
        else
            printf 'FAIL: ROS 2 包不可用: %s\n' "${package_name}" >&2
            failed=1
        fi
    done

    if python3 -c 'import nav2_simple_commander, multi_map_navigation, multi_map_navigation_msgs' \
        >/dev/null 2>&1; then
        printf 'OK: Python 导入 nav2_simple_commander, multi_map_navigation, multi_map_navigation_msgs\n'
    else
        printf 'FAIL: Python 无法导入导航运行依赖\n' >&2
        failed=1
    fi

    return "${failed}"
}

# Generated install files may change after colcon build. This refreshes the
# current shell explicitly; opening a new terminal performs the same load.
car_ros_env_reload() {
    local env_script="/home/akun/workspace/Car_jetson/navigation_tools/ros2_navigation_env.bash"
    unset CAR_JETSON_ROS_ENV_LOADED_FROM
    # shellcheck disable=SC1090
    source "${env_script}"
}

if (( _car_ros_env_had_nounset )); then
    set -u
fi

unset _car_ros_env_base _car_ros_env_had_nounset _car_ros_env_missing \
    _car_ros_env_overlays _car_ros_env_relative _car_ros_env_root \
    _car_ros_env_script _car_ros_env_setup

