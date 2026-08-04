#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K8 路点跟随脚本
用于在 LIO-SAM 和 Navigation2 启动后，发布一系列路点给导航系统
支持循环模式和单次模式
使用 ActionClient 直接与 Nav2 通信，不依赖 AMCL
"""

import rclpy
import time
import math
import os
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from tf_transformations import quaternion_from_euler


def euler_to_quaternion(yaw_degrees):
    """
    将欧拉角（度）转换为四元数

    Args:
        yaw_degrees: 偏航角（度），0°=东，90°=北，180°=西，270°=南

    Returns:
        (x, y, z, w): 四元数的四个分量
    """
    q = quaternion_from_euler(0, 0, math.radians(yaw_degrees))
    return q[0], q[1], q[2], q[3]


def load_waypoints_from_file(file_path):
    """
    从文件中加载路点数据
    文件格式: x y z angle°

    Args:
        file_path: 路点文件路径

    Returns:
        路点列表 [[x, y, qx, qy, qz, qw], ...]
    """
    waypoints = []
    if not os.path.exists(file_path):
        print(f"警告: 路点文件不存在: {file_path}")
        return waypoints

    with open(file_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            try:
                parts = line.split()
                if len(parts) < 4:
                    print(f"警告: 第{line_num}行格式错误，跳过")
                    continue

                x = float(parts[0])
                y = float(parts[1])
                # parts[2] 是z坐标，暂时不使用
                angle_str = parts[3].replace('°', '')
                angle = float(angle_str)

                # 转换角度为四元数
                qx, qy, qz, qw = euler_to_quaternion(angle)
                waypoints.append([x, y, qx, qy, qz, qw])

            except ValueError as e:
                print(f"警告: 第{line_num}行解析失败: {e}")
                continue

    return waypoints


def create_pose(x, y, qx, qy, qz, qw, frame_id='map', stamp=None):
    """
    创建一个 PoseStamped 消息

    Args:
        x: X坐标
        y: Y坐标
        qx: 四元数x分量
        qy: 四元数y分量
        qz: 四元数z分量
        qw: 四元数w分量
        frame_id: 参考坐标系，默认为'map'
        stamp: 时间戳

    Returns:
        PoseStamped对象
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp:
        pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class WaypointNavigator(Node):
    """路点导航器，使用 ActionClient 直接与 Nav2 通信"""

    def __init__(self, waypoints_data, loop_mode=False, navigation_timeout=1800.0):
        super().__init__('waypoint_navigator')

        self.waypoints_data = waypoints_data
        self.loop_mode = loop_mode
        self.navigation_timeout = navigation_timeout

        # 创建 FollowWaypoints action 客户端
        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

        self.goal_handle = None
        self.result_future = None
        self.feedback = None

        self.get_logger().info('路点导航器已初始化')

    def wait_for_nav2_server(self, timeout_sec=50.0):
        """等待 Nav2 action server 可用"""
        self.get_logger().info('等待 follow_waypoints action server...')

        if not self.follow_waypoints_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('follow_waypoints action server 不可用')
            return False

        self.get_logger().info('follow_waypoints action server 已就绪')
        return True

    def build_goal_poses(self):
        """构建路点目标列表"""
        goal_poses = []
        current_time = self.get_clock().now().to_msg()

        for i, wp in enumerate(self.waypoints_data):
            pose = create_pose(wp[0], wp[1], wp[2], wp[3], wp[4], wp[5], stamp=current_time)
            goal_poses.append(pose)
            self.get_logger().info(
                f'路点 {i+1}: x={wp[0]:.2f}, y={wp[1]:.2f}, '
                f'qx={wp[2]:.3f}, qy={wp[3]:.3f}, qz={wp[4]:.3f}, qw={wp[5]:.3f}'
            )

        return goal_poses

    def send_waypoints(self):
        """发送路点跟随目标"""
        # 构建目标消息
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.build_goal_poses()

        self.get_logger().info(f'发送 {len(goal_msg.poses)} 个路点到 Nav2...')

        # 发送目标
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.get_logger().info('等待目标被接受...')

        # 等待目标被接受（带超时）
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        if not send_goal_future.done():
            self.get_logger().error('发送目标超时')
            return False

        self.goal_handle = send_goal_future.result()

        if self.goal_handle is None:
            self.get_logger().error('未能获取目标句柄')
            return False

        if not self.goal_handle.accepted:
            self.get_logger().error('路点目标被拒绝')
            return False

        self.get_logger().info('路点目标已被接受，开始导航...')

        # 获取结果
        self.result_future = self.goal_handle.get_result_async()
        return True

    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        self.feedback = feedback_msg.feedback
        current_wp = self.feedback.current_waypoint + 1
        total_wp = len(self.waypoints_data)
        self.get_logger().info(f'正在前往路点: {current_wp}/{total_wp}')

    def wait_for_result(self):
        """等待导航结果"""
        start_time = self.get_clock().now()
        last_log_time = start_time

        self.get_logger().info('等待导航完成...')

        while not self.result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

            # 每5秒打印一次状态
            current_time = self.get_clock().now()
            elapsed = (current_time - last_log_time).nanoseconds / 1e9
            if elapsed > 5.0:
                total_elapsed = (current_time - start_time).nanoseconds / 1e9
                self.get_logger().info(f'导航进行中... 已用时: {total_elapsed:.1f}秒')
                last_log_time = current_time

            # 检查超时
            total_elapsed = (current_time - start_time).nanoseconds / 1e9
            if total_elapsed > self.navigation_timeout:
                self.get_logger().warn('导航超时，取消任务...')
                self.cancel_navigation()
                return None

        result = self.result_future.result()
        total_elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
        self.get_logger().info(f'导航结束，总用时: {total_elapsed:.1f}秒')
        return result

    def cancel_navigation(self):
        """取消当前导航"""
        if self.goal_handle:
            self.get_logger().info('正在取消导航...')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)

    def get_result_status(self, result):
        """获取结果状态"""
        if result is None:
            return 'TIMEOUT'

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            return 'CANCELED'
        elif status == GoalStatus.STATUS_ABORTED:
            return 'FAILED'
        else:
            return 'UNKNOWN'


def main():
    # ============ 配置区域 ============
    # 是否启用循环模式（True=循环导航，False=单次导航）
    LOOP_MODE = False

    # 导航超时时间（秒）
    NAVIGATION_TIMEOUT = 1800.0  # 30分钟

    # 路点文件路径
    WAYPOINT_FILE = '/home/akun/workspace/Car_jetson/K8_point2.txt'

    # 从文件加载路点，如果文件不存在或为空，则使用默认路点
    waypoints_data = load_waypoints_from_file(WAYPOINT_FILE)

    if not waypoints_data:
        print("未能从文件加载路点，使用默认路点")
        # 默认路点列表 [x, y, qx, qy, qz, qw]
        qx1, qy1, qz1, qw1 = euler_to_quaternion(0)
        qx2, qy2, qz2, qw2 = euler_to_quaternion(45)
        qx3, qy3, qz3, qw3 = euler_to_quaternion(90)
        qx4, qy4, qz4, qw4 = euler_to_quaternion(180)
        waypoints_data = [
            [1.0, 2.0, qx1, qy1, qz1, qw1],
            [3.0, 2.0, qx2, qy2, qz2, qw2],
            [3.0, 5.0, qx3, qy3, qz3, qw3],
            [1.0, 5.0, qx4, qy4, qz4, qw4],
        ]
    # ==================================

    # 初始化ROS2
    rclpy.init()

    print("=" * 50)
    print("K8 路点跟随脚本 (基于 ActionClient)")
    print("=" * 50)
    print(f"循环模式: {'开启' if LOOP_MODE else '关闭'}")
    print(f"路点数量: {len(waypoints_data)}")
    print(f"导航超时: {NAVIGATION_TIMEOUT}秒")
    print("=" * 50)

    # 创建导航器节点
    navigator = WaypointNavigator(
        waypoints_data=waypoints_data,
        loop_mode=LOOP_MODE,
        navigation_timeout=NAVIGATION_TIMEOUT
    )

    # 等待 Nav2 服务器可用
    if not navigator.wait_for_nav2_server(timeout_sec=60.0):
        print("错误: Nav2 服务器长时间未响应，请检查 Navigation2 是否正常启动")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    # 导航循环
    loop_count = 0
    try:
        while True:
            loop_count += 1
            if LOOP_MODE:
                print(f"\n{'='*50}")
                print(f"开始第 {loop_count} 轮导航")
                print(f"{'='*50}")
            else:
                print(f"\n{'='*50}")
                print("开始导航")
                print(f"{'='*50}")

            # 发送路点
            if not navigator.send_waypoints():
                print("✗ 发送路点失败")
                break

            # 等待结果
            result = navigator.wait_for_result()
            status = navigator.get_result_status(result)

            print(f"\n{'='*50}")
            if status == 'SUCCEEDED':
                print("✓ 导航成功完成!")
            elif status == 'CANCELED':
                print("✗ 导航被取消")
            elif status == 'FAILED':
                print("✗ 导航失败")
            elif status == 'TIMEOUT':
                print("✗ 导航超时")
            else:
                print("? 导航状态未知")
            print(f"{'='*50}\n")

            # 如果不是循环模式，退出
            if not LOOP_MODE:
                print("单次导航完成，退出程序")
                break

            # 导航失败，退出循环
            if status in ['FAILED', 'TIMEOUT']:
                print("导航失败，退出循环")
                break

            # 循环模式下，等待一段时间后继续
            print("等待3秒后开始下一轮...")
            time.sleep(3.0)

    except KeyboardInterrupt:
        print("\n\n接收到中断信号，停止导航...")
        navigator.cancel_navigation()

    finally:
        # 清理资源
        print("关闭导航器...")
        navigator.destroy_node()
        rclpy.shutdown()
        print("程序已退出")


if __name__ == '__main__':
    main()
