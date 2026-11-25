# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch one mux, one converter and the relative-pose Dock Action server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Require custom YAML for measured mechanical parameters."""
    dock_share = get_package_share_directory('charging_dock')
    ecu_share = get_package_share_directory('cmdvel_2_ecu')
    config = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            dock_share, 'config', 'relative_docking.yaml')),
        Node(package='charging_dock', executable='command_mux',
             name='command_mux', parameters=[config], output='screen'),
        Node(package='charging_dock', executable='relative_docking_controller',
             name='relative_docking_controller', parameters=[config], output='screen'),
        Node(package='cmdvel_2_ecu', executable='cmdvel_2_ecu',
             name='cmdvel_to_ecu', output='screen', parameters=[
                 os.path.join(ecu_share, 'config', 'config.yaml'),
                 {'cmd_vel_topic': '/cmd_vel_selected', 'docking_mode': False,
                  'docking_max_speed': 0.10, 'docking_max_steer_deg': 8.0,
                  'docking_ecu_speed_scale': 3.6, 'docking_stop_threshold': 0.001,
                  'input_timeout_sec': 0.20}]),
    ])
