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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _shutdown_if_node_exits(node, label):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[
                LogInfo(msg=f'{label} exited; stopping AprilTag docking launch.'),
                EmitEvent(event=Shutdown(reason=f'{label} exited')),
            ],
        )
    )


def generate_launch_description():
    package_share = get_package_share_directory('charging_dock')
    detector_params = LaunchConfiguration('detector_params_file')
    video_device = LaunchConfiguration('video_device')

    rear_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{package_share}/launch/rear_camera.launch.py'
        ),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={'video_device': video_device}.items(),
    )
    detector = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        namespace='apriltag',
        name='apriltag',
        output='screen',
        parameters=[detector_params],
        remappings=[
            ('image_rect', '/rear_camera/image_rect'),
            ('camera_info', '/rear_camera/camera_info'),
        ],
    )
    adapter = Node(
        package='charging_dock',
        executable='dock_tag_adapter',
        namespace='charging_dock',
        name='dock_tag_adapter',
        output='screen',
        emulate_tty=True,
        parameters=[detector_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the Stage 2 rear camera and rectification nodes.',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value=(
                '/dev/v4l/by-path/'
                'pci-0000:00:14.0-usb-0:1.1:1.0-video-index0'
            ),
            description='Stable V4L2 device path passed to rear_camera.launch.py.',
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=f'{package_share}/config/apriltag_detector.yaml',
            description='Shared AprilTag detector and adapter parameters.',
        ),
        rear_camera,
        detector,
        adapter,
        _shutdown_if_node_exits(detector, 'apriltag detector'),
        _shutdown_if_node_exits(adapter, 'dock tag adapter'),
    ])
