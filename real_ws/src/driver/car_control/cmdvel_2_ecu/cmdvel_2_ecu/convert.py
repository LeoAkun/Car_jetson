import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yunle_msgs.msg import Ecu
from std_msgs.msg import Header
import math

class CmdVelToEcu(Node):
    def __init__(self):
        super().__init__("cmdvel_to_ecu")

        # 创建发布者和订阅者
        self.ecu_pub = self.create_publisher(Ecu, "/ecu", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        # 声明参数
        self.declare_parameter('L', 0.5)
        self.declare_parameter('MAX_STEER_DEG', 20.0)
        self.declare_parameter('LOW_SPEED_THRESH', 0.3)
        self.declare_parameter('Vel_Offeset', 0.05)
        self.declare_parameter('OMEGA_DEADZONE', 0.20)
        self.declare_parameter('OMEGA_FULL_STEER', 1.20)
        self.declare_parameter('rear_wheel_flag', True)
        self.declare_parameter('STOP_THRESH', 0.02)
        
        self.declare_parameter('Vel_RATE', 1.2)

        # 读取参数
        self.L = self.get_parameter('L').value
        self.MAX_STEER_DEG = self.get_parameter('MAX_STEER_DEG').value
        self.LOW_SPEED_THRESH = self.get_parameter('LOW_SPEED_THRESH').value
        self.Vel_Offeset = self.get_parameter('Vel_Offeset').value
        self.OMEGA_DEADZONE = self.get_parameter('OMEGA_DEADZONE').value
        self.OMEGA_FULL_STEER = self.get_parameter('OMEGA_FULL_STEER').value
        self.rear_wheel_flag = self.get_parameter('rear_wheel_flag').value
        self.STOP_THRESH = self.get_parameter('STOP_THRESH').value
        self.get_logger().info("/cmd_vel → /ecu 节点已启动")

    def cmdvel_2_ecu(self, msg: Twist) -> Ecu:
        ecu_msg = Ecu()
        epsilon = 1e-6

        # 消息头
        ecu_msg.header = Header()
        ecu_msg.header.stamp = self.get_clock().now().to_msg()
        ecu_msg.header.frame_id = "base_link"

        # 获取速度与角速度
        if msg.linear.x >= self.STOP_THRESH:
            if msg.linear.x > 0:
                v = msg.linear.x + self.Vel_Offeset
            else:
                v = msg.linear.x - self.Vel_Offeset
        else:
            v = msg.linear.x

        omega = -msg.angular.z

        # ====================== 档位处理 ======================
        if v > 0:
            ecu_msg.shift = Ecu.SHIFT_D
        elif v < 0:
            ecu_msg.shift = Ecu.SHIFT_R
        else:
            ecu_msg.shift = Ecu.SHIFT_N

        # ====================== 低速限幅（保留正负号） ======================
        if abs(v) <= self.STOP_THRESH:
            # 速度极小（接近停止指令），强制归零，不补偿死区
            v = 0.0
        elif abs(v) <= self.LOW_SPEED_THRESH:
            v = self.LOW_SPEED_THRESH if v > 0 else -self.LOW_SPEED_THRESH

        # 设置电机输出
        ecu_msg.motor = abs(v)

        # ====================== 转向角计算 ======================
        if abs(v) <= epsilon:
            # 原地转向
            if abs(omega) < self.OMEGA_DEADZONE:
                steer_deg = 0.0
            else:
                steer_deg = omega * (self.MAX_STEER_DEG / self.OMEGA_FULL_STEER)
        else:
            # 行驶转向（阿克曼）
            steer_deg = math.degrees(math.atan(self.L * omega / v))

        # 转向角限幅
        ecu_msg.steer = max(-self.MAX_STEER_DEG, min(steer_deg, self.MAX_STEER_DEG))

        # ====================== 刹车与标志位 ======================
        ecu_msg.brake = abs(v) < epsilon
        ecu_msg.rear_wheel_flag = self.rear_wheel_flag

        return ecu_msg

    def cmd_callback(self, msg: Twist):
        ecu_msg = self.cmdvel_2_ecu(msg)
        self.ecu_pub.publish(ecu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToEcu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()