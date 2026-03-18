#!/usr/bin/env python3
"""
假的 NavigateToPose Action Server(先成功，然后失败)
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
            self.test_execute_callback
        )
        self.count = 0
        self.get_logger().info("假 Nav2 action server 已启动，第一次成功，第二次失败")

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"收到目标：{goal_handle.request.pose.pose.position.x:.2f}, "
                               f"{goal_handle.request.pose.pose.position.y:.2f}")

        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = 5.0
        goal_handle.publish_feedback(feedback)

        time.sleep(60.0)  # 模拟导航耗时

        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result
    
    #模拟返回终止
    def execute_callback_abort(self, goal_handle):
        self.get_logger().info(f"收到目标：{goal_handle.request.pose.pose.position.x:.2f}, "
                               f"{goal_handle.request.pose.pose.position.y:.2f}")
        self.get_logger().info("将返回终止")
        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = 5.0
        goal_handle.publish_feedback(feedback)

        time.sleep(3.0)  # 模拟导航耗时

        goal_handle.abort()
        result = NavigateToPose.Result()
        return result
    
    #模拟先成功，后失败
    def test_execute_callback(self, goal_handle):
        self.get_logger().info(f"收到目标：{goal_handle.request.pose.pose.position.x:.2f}, "
                               f"{goal_handle.request.pose.pose.position.y:.2f}")

        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = 5.0
        goal_handle.publish_feedback(feedback)

        time.sleep(2.0)  # 模拟导航耗时
        if self.count == 2: # 第三个点失败
            goal_handle.abort()

        else:
            goal_handle.succeed()
        
        self.count += 1

        result = NavigateToPose.Result()
        return result
def main():
    rclpy.init()
    node = FakeNav2Server()
    rclpy.spin(node)


if __name__ == '__main__':
    main()