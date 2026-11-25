import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ground_dir = get_package_share_directory('ground')

    params_file = LaunchConfiguration('params_file')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_front_depth_camera_publisher',
        output='screen',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'front_depth_camera_link',
        ],
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ground_dir,
                'launch',
                'pothole_detector.launch.py',
            )
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                ground_dir,
                'config',
                'pothole_detector.yaml',
            ),
            description='Pothole detector parameter file',
        ),
        DeclareLaunchArgument('camera_x', default_value='0.470'),
        DeclareLaunchArgument('camera_y', default_value='-0.010'),
        DeclareLaunchArgument('camera_z', default_value='0.005'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument(
            'camera_pitch',
            default_value='0.2617993878',
            description='Downward camera pitch, radians',
        ),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        camera_tf,
        detector,
    ])
