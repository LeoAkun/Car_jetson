import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yunle_msgs.msg import Ecu
from std_msgs.msg import Header
import math

class CmdVelToEcu(Node):
    def __init__(self):
        super().__init__("cmdvel_to_ecu")

        self.ecu_pub = self.create_publisher(Ecu, "/ecu", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        self.declare_parameter('L', 0.5)
        self.declare_parameter('MAX_STEER_DEG', 20.0)
        self.declare_parameter('LOW_SPEED_THRESH', 0.08)
        self.declare_parameter('OMEGA_DEADZONE', 0.20)
        self.declare_parameter('OMEGA_FULL_STEER', 1.20)
        self.declare_parameter('rear_wheel_flag', True)

        self.L = self.get_parameter('L').value
        self.MAX_STEER_DEG = self.get_parameter('MAX_STEER_DEG').value
        self.LOW_SPEED_THRESH = self.get_parameter('LOW_SPEED_THRESH').value
        self.OMEGA_DEADZONE = self.get_parameter('OMEGA_DEADZONE').value
        self.OMEGA_FULL_STEER = self.get_parameter('OMEGA_FULL_STEER').value
        self.rear_wheel_flag = self.get_parameter('rear_wheel_flag').value
        
        self.get_logger().info("/cmd_vel → /ecu（双阿克曼）已启动")

    def cmdvel_2_ecu(self, msg: Twist) -> Ecu:

        ecu_msg = Ecu()

        # Header
        ecu_msg.header = Header()
        ecu_msg.header.stamp = self.get_clock().now().to_msg()
        ecu_msg.header.frame_id = "base_link"

        # 速度（直接塞 motor）
        v = msg.linear.x
        
        # 档位逻辑
        if v > 0:
            ecu_msg.shift = Ecu.SHIFT_D
        elif v < 0:
            ecu_msg.shift = Ecu.SHIFT_R
        else:
            ecu_msg.shift = Ecu.SHIFT_N
        ecu_msg.motor = math.fabs(v)

        # 角速度
        omega =  - msg.angular.z

        # 如果是原地打方向
        if abs(v) <= self.LOW_SPEED_THRESH:
            if abs(omega) < self.OMEGA_DEADZONE:
                steer_deg = 0.0
            else:
                steer_deg = omega * (self.MAX_STEER_DEG / self.OMEGA_FULL_STEER)
        # 如果是边走边打方向
        else:
            steer_deg = math.degrees(math.atan(self.L * omega / v))

        # 最终限幅
        ecu_msg.steer = max(-self.MAX_STEER_DEG, min(steer_deg, self.MAX_STEER_DEG))

        # 其他 flag
        if abs(v) == 0:
            ecu_msg.brake = True
        else :
            ecu_msg.brake = False
        ecu_msg.rear_wheel_flag = self.rear_wheel_flag

        return ecu_msg

    def cmd_callback(self, msg: Twist):
        
        # 消息格式转换
        ecu_msg = self.cmdvel_2_ecu(msg)

        # 发布话题
        self.ecu_pub.publish(ecu_msg)



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToEcu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
