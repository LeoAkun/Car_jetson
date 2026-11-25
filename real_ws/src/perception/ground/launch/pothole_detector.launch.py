import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ground_dir = get_package_share_directory('ground')
    default_params_file = os.path.join(
        ground_dir,
        'config',
        'pothole_detector.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the pothole detector parameter file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        Node(
            package='ground',
            executable='height_map_pothole_detector',
            name='height_map_pothole_detector',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
