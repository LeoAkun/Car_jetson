#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# 替换成你实际的 srv 包名和服务类型
# 例如：如果 srv 文件在 my_localization/srv/SetInitialPose.srv
# 则改为：from my_localization.srv import SetInitialPose
from re_localization.srv import ReLocalization      # ← 这里改成实际名称，例如 SetInitialPose

from geometry_msgs.msg import PoseWithCovarianceStamped


class SetInitialPoseClient(Node):
    def __init__(self):
        super().__init__('set_initial_pose_client')

        # 创建客户端
        # 第二个参数是服务名称（ros2 service list 查看），第三个是 srv 类型
        self.client = self.create_client(ReLocalization, '/re_localization')  # ← 修改服务名

        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务暂不可用，等待中... (按 Ctrl+C 退出)')
            if not rclpy.ok():
                sys.exit(1)

        self.get_logger().info('已连接到服务！')

    def send_request(self, x=0.0, y=0.0, yaw_deg=0.0):
        """发送初始位姿请求"""
        req = ReLocalization.Request()

        # 填充 request 中的 initial_pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # 通常是 map 坐标系

        # 位置
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        # 朝向（yaw 转四元数）
        from math import sin, cos, radians
        yaw_rad = radians(yaw_deg)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = sin(yaw_rad / 2.0)
        pose_msg.pose.pose.orientation.w = cos(yaw_rad / 2.0)

        # 协方差（示例值：位置 ±0.5m，朝向 ±20°）
        # 36 个元素的数组，按行优先顺序
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942   # ≈ (20°)^2 in rad
        ]

        req.initial_pose = pose_msg

        self.get_logger().info(f'发送初始位姿 → x={x:.2f}m, y={y:.2f}m, yaw={yaw_deg}°')

        # 同步调用（阻塞等待结果）
        future = self.client.call_async(req)

        # 等待结果（超时 5 秒）
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info('服务调用成功！')
            self.get_logger().info(f'  success     : {response.success}')
            self.get_logger().info(f'  fitness_score: {response.fitness_score:.4f}')
            
            p = response.pose.pose.pose.position
            o = response.pose.pose.pose.orientation
            self.get_logger().info(f'  返回位姿 → x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')
            self.get_logger().info(f'             qx={o.x:.4f}, qy={o.y:.4f}, qz={o.z:.4f}, qw={o.w:.4f}')
        else:
            self.get_logger().error('服务调用失败或超时')


def main(args=None):
    rclpy.init(args=args)

    client_node = SetInitialPoseClient()

    try:
        # 示例：发送 (x=1.5, y=-0.8, yaw=45°) 的初始位姿
        # 你可以修改这里的数值，或通过命令行参数传入
        client_node.send_request(x=0.0, y=-0.0, yaw_deg=0.0)

        # 如果想多次测试，可以循环调用
        # while rclpy.ok():
        #     client_node.send_request(x=..., y=..., yaw_deg=...)
        #     time.sleep(2.0)

    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()