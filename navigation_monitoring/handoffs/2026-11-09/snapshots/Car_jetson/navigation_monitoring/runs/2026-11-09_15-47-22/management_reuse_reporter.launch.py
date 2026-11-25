from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav_config=os.path.join(get_package_share_directory('multi_map_navigation'),'config','navigation_config.yaml')
    mqtt_config='/home/akun/workspace/Car_jetson/navigation_monitoring/field_test_mqtt_local.yaml'
    return LaunchDescription([
        Node(package='multi_map_navigation',executable='process_manager',name='process_manager',output='screen',parameters=[nav_config],emulate_tty=True),
        Node(package='multi_map_navigation',executable='mqtt_task_receiver',name='mqtt_task_receiver',output='screen',parameters=[mqtt_config],emulate_tty=True),
        Node(package='multi_map_navigation',executable='route_planner',name='route_planner',output='screen',emulate_tty=True),
        Node(package='multi_map_navigation',executable='navigation_manager',name='navigation_manager',output='screen',parameters=[nav_config],emulate_tty=True),
        Node(package='multi_map_navigation',executable='map_switch_controller',name='map_switch_controller',output='screen',parameters=[nav_config],emulate_tty=True),
    ])
