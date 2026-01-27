import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class CmdVelSmoother(Node):
    def __init__(self):
        super().__init__('cmd_vel_smoother')

        # 参数：最大加速度/减速度
        self.declare_parameter('max_lin_acc', 1.0)   # m/s^2
        self.declare_parameter('max_ang_acc', 2.0)   # rad/s^2
        self.declare_parameter('rate', 30)           # Hz

        self.max_lin_acc = self.get_parameter('max_lin_acc').value
        self.max_ang_acc = self.get_parameter('max_ang_acc').value
        self.dt = 1.0 / self.get_parameter('rate').value

        # 当前和目标速度
        self.current_lin = 0.0
        self.current_ang = 0.0
        self.target_lin = 0.0
        self.target_ang = 0.0

        # 订阅 /cmd_vel，发布 /cmd_vel_smooth
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(
            Twist, '/cmd_vel_smooth', 10)

        # 定时器循环更新
        self.timer = self.create_timer(self.dt, self.update)

    def cmd_vel_callback(self, msg: Twist):
        """保存目标速度"""
        self.target_lin = msg.linear.x
        self.target_ang = msg.angular.z

    def update(self):
        """逐步逼近目标速度，避免硬刹车"""
        # 线速度
        diff_lin = self.target_lin - self.current_lin
        max_step_lin = self.max_lin_acc * self.dt
        step_lin = math.copysign(min(abs(diff_lin), max_step_lin), diff_lin)
        self.current_lin += step_lin

        # 角速度
        diff_ang = self.target_ang - self.current_ang
        max_step_ang = self.max_ang_acc * self.dt
        step_ang = math.copysign(min(abs(diff_ang), max_step_ang), diff_ang)
        self.current_ang += step_ang

        # 发布平滑后的 cmd_vel
        smooth_msg = Twist()
        smooth_msg.linear.x = self.current_lin
        smooth_msg.angular.z = self.current_ang
        self.pub.publish(smooth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
