#!/usr/bin/env python3
"""
假的 NavigateToPose Action Server（总是成功）
用于测试 NavigationManager 的 goal 发送与结果处理逻辑
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import time


class FakeNav2Server(Node):
    def __init__(self):
        super().__init__('fake_navigate_to_pose_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        self.get_logger().info("假 Nav2 action server 已启动，任何 goal 都会在3秒后成功返回")

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"收到目标：{goal_handle.request.pose.pose.position.x:.2f}, "
                               f"{goal_handle.request.pose.pose.position.y:.2f}")

        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = 5.0
        goal_handle.publish_feedback(feedback)

        time.sleep(3.0)  # 模拟导航耗时

        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result


def main():
    rclpy.init()
    node = FakeNav2Server()
    rclpy.spin(node)


if __name__ == '__main__':
    main()