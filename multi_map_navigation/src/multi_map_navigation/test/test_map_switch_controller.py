#!/usr/bin/env python3
"""
测试 MapSwitchController 的 MultiThreadedExecutor + ReentrantCallbackGroup 修复

模拟 process_manager 的三个服务，发送地图切换触发消息，
验证 map_switch_controller 能在回调内成功调用服务并完成整个切换流程。
"""

import unittest
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from multi_map_navigation_msgs.msg import (
    LocalizationResult,
    LocalizationSession,
    MapSwitchResult,
    MapSwitchTrigger,
)
from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess, GetProcessStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MockProcessManager(Node):
    """模拟 process_manager 节点，提供三个服务"""

    def __init__(self):
        super().__init__('mock_process_manager')

        self.cb_group = ReentrantCallbackGroup()

        # 跟踪进程运行状态
        self.running = {
            're_localization': True,
            'liosam': True,
            'nav2_init_pose': True,
            'navigation2': True,
        }

        # 记录服务调用历史
        self.shutdown_calls = []
        self.start_calls = []
        self.status_calls = 0
        self.session_id = 'map-switch-test-session'
        self.session_start_ns = 0
        self.active_map = ''

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.session_pub = self.create_publisher(
            LocalizationSession, '/localization_session', latched_qos)
        self.result_pub = self.create_publisher(
            LocalizationResult, '/localization_result', latched_qos)

        self.shutdown_srv = self.create_service(
            ShutdownProcess,
            '/process_manager/shutdown_process',
            self.handle_shutdown,
            callback_group=self.cb_group
        )

        self.start_srv = self.create_service(
            StartProcess,
            '/process_manager/start_process',
            self.handle_start,
            callback_group=self.cb_group
        )

        self.status_srv = self.create_service(
            GetProcessStatus,
            '/process_manager/get_status',
            self.handle_status,
            callback_group=self.cb_group
        )

        self.get_logger().info('MockProcessManager 就绪')

    def handle_shutdown(self, request, response):
        name = request.process_name
        self.get_logger().info(f'收到关闭请求: {name}')
        self.shutdown_calls.append(name)
        if name in self.running:
            self.running[name] = False
        response.success = True
        response.message = f'{name} 已关闭'
        return response

    def handle_start(self, request, response):
        name = request.process_name
        self.get_logger().info(f'收到启动请求: {name}, map={request.map_name}')
        self.start_calls.append(name)
        if name in self.running:
            self.running[name] = True
        response.success = True
        response.message = f'{name} 已启动'
        if name == 're_localization':
            self.session_start_ns = self.get_clock().now().nanoseconds
            self.active_map = request.map_name
            response.instance_id = self.session_id
            response.active_map = request.map_name
            response.session_start_ns = self.session_start_ns
            session = LocalizationSession()
            session.session_id = self.session_id
            session.map_id = request.map_name
            session.session_start_ns = self.session_start_ns
            session.active = True
            self.session_pub.publish(session)
        elif name == 'nav2_init_pose':
            response.instance_id = self.session_id
            response.active_map = request.map_name
            response.session_start_ns = self.session_start_ns
            result = LocalizationResult()
            result.session_id = self.session_id
            result.map_id = request.map_name
            result.success = True
            result.fitness_score = 0.01
            result.pose.orientation.w = 1.0
            stamp = self.get_clock().now().to_msg()
            result.cloud_stamp = stamp
            self.result_pub.publish(result)
        return response

    def handle_status(self, request, response):
        self.status_calls += 1
        response.re_localization_running = self.running['re_localization']
        response.liosam_running = self.running['liosam']
        response.nav2_init_pose_running = self.running['nav2_init_pose']
        response.navigation2_running = self.running['navigation2']
        response.message = 'ok'
        running_values = tuple(self.running.values())
        response.stack_state = (
            'RUNNING' if all(running_values)
            else 'STOPPED' if not any(running_values) else 'PARTIAL')
        response.active_map = self.active_map if all(running_values) else ''
        response.localization_session_id = (
            self.session_id if all(running_values) else '')
        response.session_start_ns = (
            self.session_start_ns if all(running_values) else 0)
        return response


class TriggerAndListener(Node):
    """发送触发消息并监听切换完成结果"""

    def __init__(self):
        super().__init__('test_trigger_listener')

        self.result = None
        self.result_event = threading.Event()

        self.trigger_pub = self.create_publisher(
            MapSwitchTrigger,
            '/trigger_map_switch',
            10
        )

        self.complete_sub = self.create_subscription(
            MapSwitchResult,
            '/map_switch_complete',
            self.on_complete,
            10
        )

    def on_complete(self, msg: MapSwitchResult):
        self.get_logger().info(f'收到切换结果: {msg.success}')
        self.result = msg.success
        self.result_event.set()

    def send_trigger(self, current_map: str, next_map: str):
        msg = MapSwitchTrigger()
        msg.transaction_id = 'integration-map-switch'
        msg.current_map = current_map
        msg.next_map = next_map
        msg.current_waypoint_id = 0
        msg.next_waypoint_id = 1
        self.trigger_pub.publish(msg)
        self.get_logger().info(f'已发送触发: {current_map} -> {next_map}')


class MockTFBroadcaster(Node):
    """
    模拟延迟发布 map -> odom TF
    真实场景中 re_localization 启动后需要数秒才能发布 TF
    """

    def __init__(self, delay_sec: float = 10.0):
        super().__init__('mock_tf_broadcaster')
        import tf2_ros
        self.br = tf2_ros.TransformBroadcaster(self)
        self.delay_sec = delay_sec
        self.published = False
        self.start_time = self.get_clock().now()

        # 用定时器延迟发布，模拟真实启动延迟
        self.timer = self.create_timer(0.1, self._publish_tf)
        self.get_logger().info(f'MockTFBroadcaster 已初始化，将在 {delay_sec}s 后发布 TF')

    def _publish_tf(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.delay_sec:
            return
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
        self.published = True
        # Keep publishing a fresh transform, matching production's unique
        # map_odom_authority heartbeat.


class TestMapSwitchController(unittest.TestCase):
    """集成测试：验证修复后的 MapSwitchController 能完成完整地图切换流程"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_full_map_switch(self):
        """
        核心测试：
        1. 启动 MockProcessManager + MockTFBroadcaster + MapSwitchController + TriggerAndListener
        2. 发送 map1->map2 切换触发
        3. 验证 /map_switch_complete 收到 True
        4. 验证服务调用顺序正确
        """
        from multi_map_navigation.map_switch_controller import MapSwitchController

        mock_pm = MockProcessManager()
        mock_tf = MockTFBroadcaster()
        controller = MapSwitchController()
        trigger = TriggerAndListener()

        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(mock_pm)
        executor.add_node(mock_tf)
        executor.add_node(controller)
        executor.add_node(trigger)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 等待节点就绪（DDS 发现需要时间）
        time.sleep(2.0)

        # 发送切换触发
        trigger.send_trigger('map1', 'map2')

        # 等待结果，超时 60 秒
        got_result = trigger.result_event.wait(timeout=60.0)

        # 验证
        self.assertTrue(got_result, '超时：未收到 /map_switch_complete 消息')
        self.assertTrue(trigger.result, '地图切换失败：收到 False')

        # 验证关闭和启动的服务都被调用了
        self.assertGreater(len(mock_pm.shutdown_calls), 0, '未调用任何关闭服务')
        self.assertGreater(len(mock_pm.start_calls), 0, '未调用任何启动服务')
        self.assertGreater(mock_pm.status_calls, 0, '未调用状态查询服务')

        # 验证启动顺序
        expected_start_order = ['re_localization', 'nav2_init_pose', 'liosam', 'navigation2']
        self.assertEqual(mock_pm.start_calls, expected_start_order,
                         f'启动顺序不正确: {mock_pm.start_calls}')

        print('\n=== 测试通过 ===')
        print(f'关闭调用: {mock_pm.shutdown_calls}')
        print(f'启动调用: {mock_pm.start_calls}')
        print(f'状态查询次数: {mock_pm.status_calls}')

        executor.shutdown()


if __name__ == '__main__':
    unittest.main()
