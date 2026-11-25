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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_camera(context):
    package_share = get_package_share_directory('charging_dock')
    camera_params = f'{package_share}/config/rear_camera.yaml'
    # usb_cam resolves symlinks relative to /dev, so resolve by-path links here.
    video_device = os.path.realpath(
        LaunchConfiguration('video_device').perform(context)
    )

    return [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='rear_camera',
            name='usb_cam',
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
            parameters=[camera_params, {'video_device': video_device}],
        ),
        Node(
            package='image_proc',
            executable='rectify_node',
            namespace='rear_camera',
            name='rear_rectify',
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect'),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value=(
                '/dev/v4l/by-path/'
                'pci-0000:00:14.0-usb-0:1.1:1.0-video-index0'
            ),
            description='Stable V4L2 path for the rear camera.',
        ),
        OpaqueFunction(function=launch_camera),
    ])
