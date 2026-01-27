import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python import get_package_share_directory
import os

import launch_ros.parameter_descriptions
def generate_launch_description():
    
    urdf_path_default = os.path.join(get_package_share_directory("robot"), 'urdf/first_robot.xacro')
    default_rviz_config_path = os.path.join(get_package_share_directory("robot"), 'rviz_config/robot_model.rviz')
    
    # 模型文件路径参数
    urdf_param = DeclareLaunchArgument(
            name = 'model',
            default_value = urdf_path_default,
            description = "加载的模型路径"
        )
    
    # 通过路径获取内容，转换为参数值对象，以供传入robot_state_publisher
    cmd_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(cmd_result, value_type=str)

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description':robot_description_value}]
        )

    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        )
    
    rviz = ExecuteProcess(
            cmd=['rviz2', '-d', default_rviz_config_path]
        )
    
    return LaunchDescription([
        urdf_param,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])