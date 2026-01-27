import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('lock_controller'),
        'config',
        'lock_controller.yaml'
    )

    # 创建节点
    lock_controller_node = Node(
        package='lock_controller',
        executable='lock_controller_node',
        name='lock_controller',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )

    return LaunchDescription([
        lock_controller_node
    ])
