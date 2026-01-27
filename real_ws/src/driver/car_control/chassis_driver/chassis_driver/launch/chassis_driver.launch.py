#encoding:utf-8
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_driver',
            executable='chassis_driver_node',
            name='chassis_driver',
            output='screen',
            parameters=[
                # 0:usb2can; 1:eth_can_udp
                {'canbus_type': 1},

                # USB2CAN
                {'main_can_id': 2},  # 1:CAN1 2:CAN2
                {'debug_can_id': 1},

                # EthCan_UDP
                {'can_eth_card': 'enp3s0'},  # 注意网口名
                {'can1_remote_ip': '192.168.1.10'},
                {'can1_remote_port': 4001},
                {'can2_remote_ip': '192.168.1.10'},
                {'can2_remote_port': 4002},
                {'local_ip': '192.168.1.140'}, # CAN盒绑定的IP
                {'local_port': 8001},

                # Common
                {'debug_mode': False},	# True or False
                {'show_sending_msg': False},	# True or False
                {'car_type': 'JD03'},       # optional: 'JD02' 'JD03' 'NWD' 'TD'
                {'vehicle_weight': 30.0}
            ]
        )
    ])
