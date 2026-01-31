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
from multi_map_navigation.process_manager import ProcessManager
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
import rclpy.duration
import tf2_ros

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
        # 关闭顺序: navigation2 -> liosam -> re_localization
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

        if not self.process_manager.shutdown_process('re_localization'):
            self.get_logger().error('关闭re_localization失败')
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
        # 启动顺序: re_localization -> nav2_init_pose -> liosam -> navigation2
        success = True

        # 启动re_localization
        if not self.process_manager.launch_relocalization(map_name):
            self.get_logger().error('启动re_localization失败')
            return False

        # 启动nav2_init_pose
        if not self.process_manager.launch_nav2_init_pose(map_name):
            self.get_logger().error('启动nav2_init_pose失败')
            self.process_manager.shutdown_process('re_localization')
            return False

        # 等待tf变换完成
        self.get_logger().info('等待重定位完成并发布 map -> odom TF...')
        if not self.wait_for_map_to_base_link_tf(timeout_sec=45.0):  # 可根据实际调整
            self.get_logger().error('map -> odom TF 长时间未出现，启动失败')
            self.process_manager.shutdown_process('nav2_init_pose')
            self.process_manager.shutdown_process('re_localization')
            return False

        # 启动liosam
        if not self.process_manager.launch_liosam():
            self.get_logger().error('启动liosam失败')
            self.process_manager.shutdown_process('nav2_init_pose')
            self.process_manager.shutdown_process('re_localization')
            return False

        # 启动navigation2
        if not self.process_manager.launch_navigation2(map_name):
            self.get_logger().error('启动navigation2失败')
            self.process_manager.shutdown_process('liosam')
            self.process_manager.shutdown_process('nav2_init_pose')
            self.process_manager.shutdown_process('re_localization')
            return False

        return True

    def wait_for_map_to_odom_link_tf(
        self,
        timeout_sec: float = 300.0, # 5分钟
        check_interval: float = 0.5
    ) -> bool:
        """
        等待 map -> odom TF 变换可用
        """
        self.get_logger().info("等待 map -> odom TF 变换就绪...")

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)

        start_time = self.get_clock().now()
        deadline = start_time + rclpy.duration.Duration(seconds=timeout_sec)

        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0)
            try:
                # 尝试查询最新的变换（time=0 表示 latest）
                trans: TransformStamped = tf_buffer.lookup_transform(
                    target_frame="map",
                    source_frame="odom",
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)  # 小超时避免阻塞
                )
                self.get_logger().info(
                    f"map -> odom TF 已就绪！ "
                    f"translation: {trans.transform.translation.x:.2f}, {trans.transform.translation.y:.2f}"
                )
                return True

            except (LookupException, ConnectivityException, ExtrapolationException):
                # 正常等待中，不报错
                self.get_logger().debug("map -> odom TF 尚未可用，继续等待...")
            
            except Exception as ex:
                self.get_logger().warn(f"TF 查询异常: {ex}")

            # 避免 CPU 空转
            time.sleep(check_interval)
        self.get_logger().error(f"等待 map -> odom TF 超时 ({timeout_sec}s)")
        return False

    def verify_stack_ready(self) -> bool:
        """
        验证导航堆栈已就绪

        返回:
            堆栈就绪返回True，否则返回False
        """
        # 检查所有进程是否正在运行
        status = self.process_manager.get_process_status()

        if not status.get('re_localization', False):
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
