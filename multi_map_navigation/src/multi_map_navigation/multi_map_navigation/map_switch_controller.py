#!/usr/bin/env python3
"""
地图切换控制器
开发者B - 导航模块

通过协调进程关闭和启动来处理地图切换操作
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from multi_map_navigation_msgs.msg import MapSwitchTrigger
from .process_manager import ProcessManager


class MapSwitchController(Node):
    def __init__(self):
        super().__init__('map_switch_controller')

        # 声明参数
        self.declare_parameter('map_switch_timeout', 30.0)
        self.declare_parameter('process_shutdown_timeout', 10.0)

        # 获取参数
        self.map_switch_timeout = self.get_parameter('map_switch_timeout').value
        self.process_shutdown_timeout = self.get_parameter('process_shutdown_timeout').value

        # 进程管理器实例
        self.process_manager = ProcessManager()

        # 当前地图跟踪
        self.current_map = None
        self.is_switching = False

        # ROS2订阅器用于地图切换触发
        self.map_switch_sub = self.create_subscription(
            MapSwitchTrigger,
            '/trigger_map_switch',
            self.map_switch_callback,
            10
        )

        # ROS2发布器用于地图切换完成
        self.map_switch_complete_pub = self.create_publisher(
            Bool,
            '/map_switch_complete',
            10
        )

        self.get_logger().info('地图切换控制器已初始化')

    def map_switch_callback(self, msg: MapSwitchTrigger):
        """
        地图切换触发回调

        参数:
            msg: 包含切换信息的MapSwitchTrigger消息
        """
        if self.is_switching:
            self.get_logger().warn('地图切换已在进行中，忽略请求')
            return

        self.get_logger().info(
            f'触发地图切换: {msg.current_map} -> {msg.next_map}'
        )

        # 执行地图切换
        success = self.execute_map_switch(msg.current_map, msg.next_map)

        # 发布完成状态
        completion_msg = Bool()
        completion_msg.data = success
        self.map_switch_complete_pub.publish(completion_msg)

        if success:
            self.get_logger().info('地图切换成功完成')
        else:
            self.get_logger().error('地图切换失败')

    def execute_map_switch(self, current_map: str, next_map: str) -> bool:
        """
        执行地图切换过程

        参数:
            current_map: 当前地图名称
            next_map: 下一个地图名称

        返回:
            切换成功返回True，否则返回False
        """
        self.is_switching = True

        try:
            # 步骤1: 关闭当前堆栈
            self.get_logger().info('步骤1: 正在关闭当前导航堆栈')
            if not self.shutdown_current_stack():
                self.get_logger().error('关闭当前堆栈失败')
                self.is_switching = False
                return False

            # 步骤2: 等待完全关闭
            self.get_logger().info('步骤2: 正在等待完全关闭')
            if not self.wait_for_clean_shutdown(self.process_shutdown_timeout):
                self.get_logger().warn('完全关闭超时，仍然继续')

            # 步骤3: 启动新堆栈
            self.get_logger().info(f'步骤3: 正在为地图启动新导航堆栈: {next_map}')
            if not self.launch_new_stack(next_map):
                self.get_logger().error('启动新堆栈失败')
                self.is_switching = False
                return False

            # 步骤4: 验证堆栈已就绪
            self.get_logger().info('步骤4: 正在验证新堆栈已就绪')
            if not self.verify_stack_ready():
                self.get_logger().error('新堆栈验证失败')
                self.is_switching = False
                return False

            # 更新当前地图
            self.current_map = next_map
            self.is_switching = False
            return True

        except Exception as e:
            self.get_logger().error(f'地图切换期间发生异常: {e}')
            self.is_switching = False
            return False

    def shutdown_current_stack(self) -> bool:
        """
        关闭当前导航堆栈

        返回:
            关闭成功返回True，否则返回False
        """
        # 关闭顺序: navigation2 -> liosam -> relocalization
        success = True

        if not self.process_manager.shutdown_process('navigation2'):
            self.get_logger().error('关闭navigation2失败')
            success = False
    
        if not self.process_manager.shutdown_process('liosam'):
            self.get_logger().error('关闭liosam失败')
            success = False

        if not self.process_manager.shutdown_process('nav2_init_pose'):
            self.get_logger().error('关闭nav2_init_pose失败')
            success = False

        if not self.process_manager.shutdown_process('relocalization'):
            self.get_logger().error('关闭relocalization失败')
            success = False

        return success

    def wait_for_clean_shutdown(self, timeout: float) -> bool:
        """
        等待所有进程完全关闭

        参数:
            timeout: 最大等待时间（秒）

        返回:
            所有进程关闭返回True，超时返回False
        """
        return self.process_manager.wait_for_clean_shutdown(timeout)

    def launch_new_stack(self, map_name: str) -> bool:
        """
        为指定地图启动新导航堆栈

        参数:
            map_name: 要启动的地图名称

        返回:
            启动成功返回True，否则返回False
        """
        # 启动顺序: relocalization -> liosam -> navigation2
        success = True

        # 启动relocalization
        if not self.process_manager.launch_relocalization(map_name):
            self.get_logger().error('启动relocalization失败')
            return False

        # 启动nav2_init_pose
        if not self.process_manager.launch_nav2_init_pose(map_name):
            self.get_logger().error('启动nav2_init_pose失败')
            self.process_manager.shutdown_process('relocalization')
            return False

        # 启动liosam
        if not self.process_manager.launch_liosam():
            self.get_logger().error('启动liosam失败')
            self.process_manager.shutdown_process('nav2_init_pose')
            self.process_manager.shutdown_process('relocalization')
            return False

        # 启动navigation2
        if not self.process_manager.launch_navigation2(map_name):
            self.get_logger().error('启动navigation2失败')
            self.process_manager.shutdown_process('liosam')
            self.process_manager.shutdown_process('nav2_init_pose')
            self.process_manager.shutdown_process('relocalization')
            return False

        return True

    def verify_stack_ready(self) -> bool:
        """
        验证导航堆栈已就绪

        返回:
            堆栈就绪返回True，否则返回False
        """
        # 检查所有进程是否正在运行
        status = self.process_manager.get_process_status()

        if not status.get('relocalization', False):
            self.get_logger().error('重定位进程未运行')
            return False

        if not status.get('liosam', False):
            self.get_logger().error('LIO-SAM进程未运行')
            return False

        if not status.get('navigation2', False):
            self.get_logger().error('Navigation2进程未运行')
            return False

        self.get_logger().info('所有进程已验证并正在运行')
        return True

    def destroy_node(self):
        """节点销毁时的清理"""
        self.process_manager.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapSwitchController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
