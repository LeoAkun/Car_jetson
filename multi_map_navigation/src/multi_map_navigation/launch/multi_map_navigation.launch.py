#!/usr/bin/env python3
"""
多地图导航启动文件
启动多地图导航系统的所有节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包目录
    pkg_dir = get_package_share_directory('multi_map_navigation')

    # 声明启动参数
    mqtt_config_arg = DeclareLaunchArgument(
        'mqtt_config',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'mqtt_config.yaml']),
        description='MQTT配置文件路径'
    )

    nav_config_arg = DeclareLaunchArgument(
        'nav_config',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'navigation_config.yaml']),
        description='导航配置文件路径'
    )

    # MQTT任务接收器节点
    mqtt_task_receiver_node = Node(
        package='multi_map_navigation',
        executable='mqtt_task_receiver',
        name='mqtt_task_receiver',
        output='screen',
        parameters=[LaunchConfiguration('mqtt_config')],
        emulate_tty=True
    )

    # 状态报告器节点
    status_reporter_node = Node(
        package='multi_map_navigation',
        executable='status_reporter',
        name='status_reporter',
        output='screen',
        parameters=[LaunchConfiguration('mqtt_config')],
        emulate_tty=True
    )

    # 导航管理器节点
    navigation_manager_node = Node(
        package='multi_map_navigation',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
        parameters=[LaunchConfiguration('nav_config')],
        emulate_tty=True
    )

    # 地图切换控制器节点
    map_switch_controller_node = Node(
        package='multi_map_navigation',
        executable='map_switch_controller',
        name='map_switch_controller',
        output='screen',
        parameters=[LaunchConfiguration('nav_config')],
        emulate_tty=True
    )

    # 进程管理器节点
    # process_manager_node = Node(
    #     package='multi_map_navigation',
    #     executable='process_manager',
    #     name='process_manager',
    #     output='screen',
    #     parameters=[LaunchConfiguration('nav_config')],
    #     emulate_tty=True
    # )

    return LaunchDescription([
        mqtt_config_arg,
        nav_config_arg,
        mqtt_task_receiver_node,
        status_reporter_node,
        navigation_manager_node,
        map_switch_controller_node,
        # process_manager_node
    ])
