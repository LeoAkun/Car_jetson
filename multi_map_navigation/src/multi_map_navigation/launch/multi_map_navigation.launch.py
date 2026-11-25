#!/usr/bin/env python3
"""
多地图导航启动文件
启动多地图导航系统的所有节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    # route_planner节点
    route_planner_node = Node(
        package='multi_map_navigation',
        executable='route_planner',
        name='route_planner',
        output='screen',
        emulate_tty=True
    )

    route_planner_exit_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=route_planner_node,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='route_planner exited; route service unavailable'))
            ],
        )
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

    navigation_manager_exit_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_manager_node,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='navigation_manager exited; task owner unavailable'))
            ],
        )
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
    process_manager_node = Node(
        package='multi_map_navigation',
        executable='process_manager',
        name='process_manager',
        output='screen',
        parameters=[LaunchConfiguration('nav_config')],
        emulate_tty=True
    )

    process_manager_exit_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=process_manager_node,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='process_manager exited; stop duplicate/unsafe managers'))
            ],
        )
    )

    # 唯一持有 map->odom 的管理节点；每轮新定位会话会先清除旧 TF。
    map_odom_authority_node = Node(
        package='multi_map_navigation',
        executable='map_odom_authority',
        name='map_odom_authority',
        output='screen',
        parameters=[LaunchConfiguration('nav_config')],
        emulate_tty=True
    )

    map_odom_authority_exit_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=map_odom_authority_node,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='map_odom_authority exited; TF owner unavailable'))
            ],
        )
    )

    return LaunchDescription([
        mqtt_config_arg,
        nav_config_arg,
        map_odom_authority_exit_guard,
        map_odom_authority_node,
        process_manager_exit_guard,
        process_manager_node,
        mqtt_task_receiver_node,
        status_reporter_node,
        route_planner_exit_guard,
        route_planner_node,
        navigation_manager_exit_guard,
        navigation_manager_node,
        map_switch_controller_node,
        
    ])
