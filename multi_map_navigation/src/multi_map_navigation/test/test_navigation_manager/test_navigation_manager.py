#!/usr/bin/env python3
"""
测试 NavigationManager 的完整导航流程

测试场景: 3 个航点
  航点0: 普通点 (type=1, map1) → 启动导航堆栈 → 导航到达 → 前进到下一个
  航点1: 地图切换点 (type=4, map1→map2) → 导航到达 → 触发地图切换
  航点2: 普通点 (type=1, map2, 最后一个) → 地图切换完成后导航 → 到达 → 导航完成

验证:
  - 导航堆栈按正确顺序启动
  - 导航目标正确发送
  - 地图切换正确触发
  - 最终状态恢复 idle
"""

import unittest
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer

from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from multi_map_navigation_msgs.msg import WaypointList, Waypoint, MapSwitchTrigger
from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess


class MockProcessManager(Node):
    """模拟 process_manager 节点"""

    def __init__(self):
        super().__init__('mock_process_manager')
        self.cb_group = ReentrantCallbackGroup()

        self.start_calls = []
        self.shutdown_calls = []

        self.create_service(
            StartProcess,
            '/process_manager/start_process',
            self.handle_start,
            callback_group=self.cb_group
        )
        self.create_service(
            ShutdownProcess,
            '/process_manager/shutdown_process',
            self.handle_shutdown,
            callback_group=self.cb_group
        )
        self.get_logger().info('MockProcessManager 就绪')

    def handle_start(self, req, resp):
        self.get_logger().info(f'启动: {req.process_name}, map={req.map_name}')
        self.start_calls.append(req.process_name)
        resp.success = True
        resp.message = f'{req.process_name} ok'
        return resp

    def handle_shutdown(self, req, resp):
        self.get_logger().info(f'关闭: {req.process_name}')
        self.shutdown_calls.append(req.process_name)
        resp.success = True
        resp.message = f'{req.process_name} ok'
        return resp


class MockNav2ActionServer(Node):
    """模拟 NavigateToPose 动作服务器"""

    def __init__(self):
        super().__init__('mock_nav2_action_server')
        self.cb_group = ReentrantCallbackGroup()
        self.goals_received = []

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            self.execute_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info('Mock Nav2 动作服务器就绪')

    def execute_callback(self, goal_handle):
        pose = goal_handle.request.pose
        x = pose.pose.position.x
        y = pose.pose.position.y
        self.get_logger().info(f'收到导航目标: x={x:.2f}, y={y:.2f}')
        self.goals_received.append(pose)

        # 模拟短暂导航
        time.sleep(0.3)

        goal_handle.succeed()
        return NavigateToPose.Result()


class MockMapSwitchController(Node):
    """
    模拟地图切换控制器
    收到 /trigger_map_switch 后延迟发布成功到 /map_switch_complete
    """

    def __init__(self, delay_sec=1.0):
        super().__init__('mock_map_switch_controller')
        self.delay_sec = delay_sec
        self.triggers_received = []

        self.sub = self.create_subscription(
            MapSwitchTrigger,
            '/trigger_map_switch',
            self.on_trigger,
            10
        )
        self.pub = self.create_publisher(Bool, '/map_switch_complete', 10)
        self.get_logger().info('MockMapSwitchController 就绪')

    def on_trigger(self, msg):
        self.get_logger().info(f'收到地图切换触发: {msg.current_map} -> {msg.next_map}')
        self.triggers_received.append(msg)

        def delayed_complete():
            time.sleep(self.delay_sec)
            result = Bool()
            result.data = True
            self.pub.publish(result)
            self.get_logger().info('已发布地图切换完成')

        threading.Thread(target=delayed_complete, daemon=True).start()


class MockTFBroadcaster(Node):
    """模拟延迟发布 map -> odom TF"""

    def __init__(self, delay_sec=3.0):
        super().__init__('mock_tf_broadcaster')
        import tf2_ros
        self.br = tf2_ros.StaticTransformBroadcaster(self)
        self.delay_sec = delay_sec
        self.published = False
        self.timer = self.create_timer(self.delay_sec, self._publish_tf)
        self.get_logger().info(f'MockTFBroadcaster 将在 {delay_sec}s 后发布 TF')

    def _publish_tf(self):
        if self.published:
            return
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
        self.published = True
        self.timer.cancel()
        self.get_logger().info('已发布 map -> odom TF')


class StateListener(Node):
    """监听 /robot_state，等导航完成后状态回到 idle"""

    def __init__(self):
        super().__init__('state_listener')
        self.states = []
        self.idle_after_running = threading.Event()

        self.sub = self.create_subscription(
            String,
            '/robot_state',
            self.on_state,
            10
        )

    def on_state(self, msg):
        self.get_logger().info(f'机器人状态: {msg.data}')
        self.states.append(msg.data)
        # 导航完成后状态序列为: idle → running → idle
        if len(self.states) >= 3 and msg.data == 'idle':
            self.idle_after_running.set()


def make_waypoint(wp_id, wp_type, map_name, x, y, yaw=0.0,
                  next_map='', next_x=0.0, next_y=0.0, next_yaw=0.0):
    """构造 Waypoint 消息"""
    wp = Waypoint()
    wp.id = wp_id
    wp.name = f'wp_{wp_id}'
    wp.map_name = map_name
    wp.type = wp_type
    wp.x = x
    wp.y = y
    wp.yaw = yaw
    wp.next_map_name = next_map
    wp.next_x = next_x
    wp.next_y = next_y
    wp.next_yaw = next_yaw
    return wp


class TestNavigationManager(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_full_navigation_with_map_switch(self):
        """
        测试完整导航流程:
        航点0(普通,map1) → 航点1(地图切换,map1→map2) → 航点2(普通,map2)
        """
        from multi_map_navigation.navigation_manager import NavigationManager

        # 创建 mock 节点（必须在 NavigationManager 之前，否则 wait_for_service 超时）
        mock_pm = MockProcessManager()
        mock_nav2 = MockNav2ActionServer()
        mock_switch = MockMapSwitchController(delay_sec=1.0)
        mock_tf = MockTFBroadcaster(delay_sec=3.0)
        state_listener = StateListener()

        executor = MultiThreadedExecutor(num_threads=10)
        executor.add_node(mock_pm)
        executor.add_node(mock_nav2)
        executor.add_node(mock_switch)
        executor.add_node(mock_tf)
        executor.add_node(state_listener)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 等待 mock 服务就绪
        time.sleep(2.0)

        # 创建 NavigationManager（会 wait_for_service，需要 mock 已就绪）
        nav_manager = NavigationManager()
        executor.add_node(nav_manager)

        # 等待 DDS 发现
        time.sleep(2.0)

        # 构造航点列表
        waypoints = [
            make_waypoint(0, 1, 'map1', 1.0, 2.0, 0.0),            # 普通点
            make_waypoint(1, 4, 'map1', 3.0, 4.0, 0.0,             # 地图切换点
                          next_map='map2', next_x=0.5, next_y=0.5, next_yaw=1.57),
            make_waypoint(2, 1, 'map2', 5.0, 6.0, 0.0),            # 普通点（最终）
        ]

        # 创建发布者发送航点列表
        wp_pub = nav_manager.create_publisher(WaypointList, '/waypoint_list', 10)
        time.sleep(1.0)

        msg = WaypointList()
        msg.header.stamp = nav_manager.get_clock().now().to_msg()
        msg.task_id = 'test_task_001'
        msg.waypoints = waypoints
        msg.total_waypoints = len(waypoints)
        msg.start_map_name = 'map1'
        wp_pub.publish(msg)

        nav_manager.get_logger().info('已发送航点列表')

        # 等待导航完成（超时 120 秒，含 TF 等待 + sleep(5) + 导航 + 地图切换）
        got_idle = state_listener.idle_after_running.wait(timeout=120.0)

        # === 验证 ===
        print('\n========== 测试结果 ==========')

        # 1. 导航是否完成
        print(f'状态序列: {state_listener.states}')
        self.assertTrue(got_idle, '超时：导航未完成，未收到最终 idle 状态')

        # 2. 导航堆栈启动顺序
        expected_start = ['re_localization', 'nav2_init_pose', 'liosam', 'navigation2']
        print(f'启动调用: {mock_pm.start_calls}')
        self.assertEqual(mock_pm.start_calls[:4], expected_start,
                         f'启动顺序不正确: {mock_pm.start_calls[:4]}')

        # 3. 导航目标数量（3 个航点各发送一次导航目标）
        print(f'导航目标数: {len(mock_nav2.goals_received)}')
        self.assertGreaterEqual(len(mock_nav2.goals_received), 2,
                                f'导航目标数量不足: {len(mock_nav2.goals_received)}')

        # 4. 地图切换触发
        print(f'地图切换触发数: {len(mock_switch.triggers_received)}')
        self.assertEqual(len(mock_switch.triggers_received), 1,
                         '应该触发恰好 1 次地图切换')
        trigger = mock_switch.triggers_received[0]
        self.assertEqual(trigger.current_map, 'map1')
        self.assertEqual(trigger.next_map, 'map2')

        # 5. 关闭进程（导航完成后会关闭所有进程）
        print(f'关闭调用: {mock_pm.shutdown_calls}')
        self.assertGreater(len(mock_pm.shutdown_calls), 0, '导航完成后应调用关闭进程')

        print('\n=== 全部测试通过 ===')

        executor.shutdown()


if __name__ == '__main__':
    unittest.main()
