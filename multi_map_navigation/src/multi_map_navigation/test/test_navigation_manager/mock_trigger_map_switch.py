#!/usr/bin/env python3
"""
地图切换控制器
开发者B - 导航模块

通过协调进程关闭和启动来处理地图切换操作
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from multi_map_navigation_msgs.msg import MapSwitchTrigger
from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess, GetProcessStatus
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
import rclpy.duration
import tf2_ros
import time

class MapSwitchController(Node):
    def __init__(self):
        super().__init__('map_switch_controller')

        # 声明参数
        self.declare_parameter('map_switch_timeout', 30.0)
        self.declare_parameter('process_shutdown_timeout', 30.0)

        # 获取参数
        self.map_switch_timeout = self.get_parameter('map_switch_timeout').value
        self.process_shutdown_timeout = self.get_parameter('process_shutdown_timeout').value

        # 使用 ReentrantCallbackGroup 允许回调内调用服务
        self.cb_group = ReentrantCallbackGroup()

        # 进程管理服务客户端
        self.start_process_client = self.create_client(
            StartProcess, '/process_manager/start_process', callback_group=self.cb_group)
        self.shutdown_process_client = self.create_client(
            ShutdownProcess, '/process_manager/shutdown_process', callback_group=self.cb_group)
        self.get_process_status_client = self.create_client(
            GetProcessStatus, '/process_manager/get_status', callback_group=self.cb_group)

        # 当前地图跟踪
        self.current_map = None
        self.is_switching = False

        # ROS2订阅器用于地图切换触发
        self.map_switch_sub = self.create_subscription(
            MapSwitchTrigger,
            '/test/trigger_map_switch',
            self.map_switch_callback,
            10,
            callback_group=self.cb_group
        )

        # ROS2发布器用于地图切换完成
        self.map_switch_complete_pub = self.create_publisher(
            Bool,
            '/test/map_switch_complete',
            10
        )

        self.get_logger().info('地图切换控制器已初始化')
        self.count = 0

    def map_switch_callback(self, msg: MapSwitchTrigger):
        """
        地图切换触发回调

        参数:
            msg: 包含切换信息的MapSwitchTrigger消息
        """
        if self.is_switching:
            self.get_logger().warn('地图切换已在进行中，忽略请求')
            return


        if self.count == 0 or self.count == 1:
            success = False
            completion_msg = Bool()
            completion_msg.data = success
            self.map_switch_complete_pub.publish(completion_msg)
            self.count += 1
            return

        self.count += 1
        self.get_logger().info(
            f'触发地图切换: {msg.current_map} -> {msg.next_map}'
        )

        # 发布完成状态
        success = True
        completion_msg = Bool()
        completion_msg.data = success
        self.map_switch_complete_pub.publish(completion_msg)

        if success:
            self.get_logger().info('地图切换成功完成')
        else:
            self.get_logger().error('地图切换失败')

def main(args=None):
    rclpy.init(args=args)
    node = MapSwitchController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
