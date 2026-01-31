#!/usr/bin/env python3
"""
模拟TF发布节点 - 用于测试地图切换逻辑
持续发布 odom->base_link 变换来模拟机器人运动
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math


class MockTFPublisher(Node):
    def __init__(self):
        super().__init__('mock_tf_publisher')

        # 创建TF广播器（注意：不是StaticTransformBroadcaster）
        self.tf_broadcaster = TransformBroadcaster(self)

        # 创建定时器，持续发布TF（20Hz）
        self.timer = self.create_timer(0.05, self.publish_tf)

        # 机器人当前状态
        self.current_x = 0.0  # 当前位置x（米）
        self.current_y = 0.0  # 当前位置y（米）
        self.current_yaw = 0.0  # 当前朝向（弧度）

        # 目标位置
        self.target_x = None
        self.target_y = None
        self.target_yaw = None

        # 运动参数
        self.linear_speed = 0.2  # 线速度（米/秒）
        self.angular_speed = 0.5  # 角速度（弧度/秒）
        self.position_tolerance = 0.1  # 位置到达容差（米）
        self.yaw_tolerance = 0.05  # 角度到达容差（弧度）

        # 订阅导航目标，获取目标点
        from nav2_msgs.action import NavigateToPose
        from action_msgs.msg import GoalStatusArray

        # 订阅导航状态，判断是否有目标
        self.navigation_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.navigation_status_callback,
            10
        )

        self.has_goal = False
        self.goal_reached = False

        self.get_logger().info('模拟TF发布节点已启动')
        self.get_logger().info('初始位置: x=0.0, y=0.0, yaw=0.0')
        self.get_logger().info('等待导航目标...')

    def navigation_status_callback(self, msg):
        """
        监听导航状态
        当有新的导航目标时，设置目标点
        """
        if len(msg.status_list) == 0:
            return

        status = msg.status_list[-1]

        # 检测到新的导航目标（状态为ACCEPTING或EXECUTING）
        if status.status == 1 or status.status == 2:  # 1=ACCEPTING, 2=EXECUTING
            if not self.has_goal:
                self.has_goal = True
                self.goal_reached = False
                self.get_logger().info('检测到新的导航目标')

        # 导航成功到达
        elif status.status == 4:  # 4=SUCCEEDED
            if self.has_goal and not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info('导航成功到达目标')

        # 导航失败或中止
        elif status.status == 5 or status.status == 6:  # 5=ABORTED, 6=CANCELED
            self.has_goal = False
            self.goal_reached = False
            self.get_logger().warn('导航已中止或取消')

    def set_target(self, x, y, yaw):
        """
        设置目标位置

        参数:
            x: 目标x坐标
            y: 目标y坐标
            yaw: 目标yaw角（弧度）
        """
        self.target_x = x
        self.target_y = y
        self.target_yaw = yaw
        self.get_logger().info(
            f'设置目标点: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

    def publish_tf(self):
        """
        持续发布TF变换，模拟机器人向目标点移动
        """
        # 如果有目标，向目标移动
        if self.has_goal and self.target_x is not None:
            self.move_towards_target()

        # 发布 odom->base_link 变换
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # 设置位置
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = 0.0

        # 设置方向
        q = quaternion_from_euler(0.0, 0.0, self.current_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 发布TF
        self.tf_broadcaster.sendTransform(t)

    def move_towards_target(self):
        """
        简单的运动控制逻辑：向目标点移动
        """
        # 计算距离误差
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # 计算角度误差
        yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)

        # 检查是否到达
        if distance < self.position_tolerance and abs(yaw_error) < self.yaw_tolerance:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(
                    f'到达目标: x={self.current_x:.2f}, '
                    f'y={self.current_y:.2f}, yaw={self.current_yaw:.2f}'
                )
            return

        # 计算目标方向
        target_yaw = math.atan2(dy, dx)
        yaw_to_target = self.normalize_angle(target_yaw - self.current_yaw)

        # 简单的控制器：先旋转朝向目标，再直线前进
        if abs(yaw_to_target) > self.yaw_tolerance:
            # 旋转朝向目标
            rotation_step = self.angular_speed * 0.05  # 0.05秒的步长
            if abs(yaw_to_target) < rotation_step:
                self.current_yaw = target_yaw
            else:
                self.current_yaw += math.copysign(rotation_step, yaw_to_target)
        else:
            # 直线前进
            move_step = min(self.linear_speed * 0.05, distance)
            self.current_x += move_step * math.cos(self.current_yaw)
            self.current_y += move_step * math.sin(self.current_yaw)

        # 最后调整朝向到目标yaw
        if distance < self.position_tolerance and abs(yaw_error) >= self.yaw_tolerance:
            rotation_step = self.angular_speed * 0.05
            if abs(yaw_error) < rotation_step:
                self.current_yaw = self.target_yaw
            else:
                self.current_yaw += math.copysign(rotation_step, yaw_error)

        # 归一化角度
        self.current_yaw = self.normalize_angle(self.current_yaw)

    @staticmethod
    def normalize_angle(angle):
        """
        将角度归一化到 [-pi, pi]

        参数:
            angle: 输入角度（弧度）

        返回:
            归一化后的角度
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MockTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n停止TF发布...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
