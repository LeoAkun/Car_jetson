#!/usr/bin/env python3
"""
测试 MapSwitchController 的完整地图切换流程

要求：
- 尽量保持真实依赖，只 mock process_manager 的 3 个服务
- 真实运行 MapSwitchController
- 通过真实 TF 广播满足 map -> odom 检查
- 发送真实地图切换触发消息并验证完成结果
"""

import threading
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from multi_map_navigation_msgs.msg import MapSwitchTrigger
from multi_map_navigation_msgs.srv import GetProcessStatus, ShutdownProcess, StartProcess
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class MockProcessManager(Node):
    """仅 mock MapSwitchController 依赖的三个 process_manager 服务"""

    def __init__(self):
        super().__init__('mock_process_manager')
        self.cb_group = ReentrantCallbackGroup()

        self.running = {
            're_localization': True,
            'liosam': True,
            'nav2_init_pose': True,
            'navigation2': True,
        }

        self.shutdown_calls = []
        self.start_calls = []
        self.status_calls = 0
        self.start_requests = []

        self.create_service(
            ShutdownProcess,
            '/process_manager/shutdown_process',
            self.handle_shutdown,
            callback_group=self.cb_group,
        )
        self.create_service(
            StartProcess,
            '/process_manager/start_process',
            self.handle_start,
            callback_group=self.cb_group,
        )
        self.create_service(
            GetProcessStatus,
            '/process_manager/get_status',
            self.handle_status,
            callback_group=self.cb_group,
        )

        self.get_logger().info('MockProcessManager 就绪')

    def handle_shutdown(self, request, response):
        process_name = request.process_name
        self.shutdown_calls.append(process_name)

        if process_name in self.running:
            self.running[process_name] = False

        response.success = True
        response.message = f'{process_name} shutdown ok'
        return response

    def handle_start(self, request, response):
        process_name = request.process_name
        self.start_calls.append(process_name)
        self.start_requests.append((process_name, request.map_name))

        if process_name in self.running:
            self.running[process_name] = True

        response.success = True
        response.message = f'{process_name} start ok'
        return response

    def handle_status(self, request, response):
        _ = request
        self.status_calls += 1
        response.re_localization_running = self.running['re_localization']
        response.liosam_running = self.running['liosam']
        response.nav2_init_pose_running = self.running['nav2_init_pose']
        response.navigation2_running = self.running['navigation2']
        response.message = 'ok'
        return response


class DelayedMapToOdomBroadcaster(Node):
    """延迟发布真实 map -> odom TF，模拟重定位完成后的 TF 可用状态"""

    def __init__(self, delay_sec: float = 1.0):
        super().__init__('delayed_map_to_odom_broadcaster')
        self.delay_sec = delay_sec
        self.tf_broadcaster = TransformBroadcaster(self)
        self.start_time = self.get_clock().now()
        self.publish_count = 0
        self.timer = self.create_timer(0.1, self.publish_tf_if_ready)

    def publish_tf_if_ready(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.delay_sec:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)
        self.publish_count += 1


class TriggerAndResultListener(Node):
    """发送地图切换触发并监听切换结果"""

    def __init__(self):
        super().__init__('trigger_and_result_listener')
        self.result = None
        self.result_event = threading.Event()

        self.trigger_pub = self.create_publisher(MapSwitchTrigger, '/trigger_map_switch', 10)
        self.complete_sub = self.create_subscription(Bool, '/map_switch_complete', self.on_complete, 10)

    def on_complete(self, msg: Bool):
        self.result = msg.data
        self.result_event.set()

    def send_trigger(self, current_map: str, next_map: str):
        msg = MapSwitchTrigger()
        msg.current_map = current_map
        msg.next_map = next_map
        msg.current_waypoint_id = 1
        msg.next_waypoint_id = 2
        self.trigger_pub.publish(msg)


class TestMapSwitchControllerRealDeps(unittest.TestCase):
    """真实依赖集成测试：仅 mock 服务，验证完整地图切换流程"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_full_map_switch_with_real_tf_and_minimal_service_mocks(self):
        from multi_map_navigation.map_switch_controller import MapSwitchController

        mock_process_manager = MockProcessManager()
        tf_broadcaster = DelayedMapToOdomBroadcaster(delay_sec=1.0)
        controller = MapSwitchController()
        trigger_listener = TriggerAndResultListener()

        controller.map_switch_timeout = 5.0
        controller.process_shutdown_timeout = 5.0

        original_sleep = time.sleep

        def patched_sleep(seconds):
            if seconds == 50.0:
                original_sleep(0.1)
            else:
                original_sleep(seconds)

        executor = MultiThreadedExecutor(num_threads=8)
        nodes = [mock_process_manager, tf_broadcaster, controller, trigger_listener]
        for node in nodes:
            executor.add_node(node)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            time.sleep(1.5)
            time.sleep = patched_sleep

            trigger_listener.send_trigger('map1', 'map2')
            got_result = trigger_listener.result_event.wait(timeout=20.0)

            self.assertTrue(got_result, '超时：未收到 /map_switch_complete 消息')
            self.assertTrue(trigger_listener.result, '地图切换失败：收到 False')

            self.assertEqual(
                mock_process_manager.shutdown_calls,
                ['navigation2', 'liosam', 'nav2_init_pose', 're_localization'],
                f'关闭顺序不正确: {mock_process_manager.shutdown_calls}',
            )
            self.assertEqual(
                mock_process_manager.start_calls,
                ['re_localization', 'nav2_init_pose', 'liosam', 'navigation2'],
                f'启动顺序不正确: {mock_process_manager.start_calls}',
            )
            self.assertGreater(mock_process_manager.status_calls, 0, '未调用状态查询服务')
            self.assertGreater(tf_broadcaster.publish_count, 0, '未发布真实 map -> odom TF')
            self.assertEqual(controller.current_map, 'map2', '当前地图未更新为目标地图')
            self.assertFalse(controller.is_switching, '地图切换结束后 is_switching 应为 False')
            self.assertEqual(
                mock_process_manager.start_requests,
                [
                    ('re_localization', 'map2'),
                    ('nav2_init_pose', 'map2'),
                    ('liosam', ''),
                    ('navigation2', 'map2'),
                ],
                f'启动请求参数不符合预期: {mock_process_manager.start_requests}',
            )
        finally:
            time.sleep = original_sleep
            executor.shutdown()
            spin_thread.join(timeout=2.0)
            for node in nodes:
                node.destroy_node()


if __name__ == '__main__':
    unittest.main()
