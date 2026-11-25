import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_no = LaunchConfiguration('serial_no')
    depth_profile = LaunchConfiguration('depth_module.depth_profile')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': 'front_depth_camera',
            'serial_no': serial_no,
            'enable_color': 'false',
            'enable_depth': 'true',
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'depth_module.depth_profile': depth_profile,
            'depth_module.depth_format': 'Z16',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
            'publish_tf': 'true',
            'initial_reset': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no',
            default_value="'117322074703'",
            description='RealSense serial number, quoted to preserve string type',
        ),
        DeclareLaunchArgument(
            'depth_module.depth_profile',
            default_value='848x480x30',
            description='RealSense depth stream profile',
        ),
        realsense_launch,
    ])
