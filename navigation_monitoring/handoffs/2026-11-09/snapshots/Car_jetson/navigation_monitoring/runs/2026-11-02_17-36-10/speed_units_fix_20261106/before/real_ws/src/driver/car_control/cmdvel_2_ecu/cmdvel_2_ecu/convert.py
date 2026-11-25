import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yunle_msgs.msg import Ecu, VehicleStatus
from std_msgs.msg import Header
from std_srvs.srv import SetBool
import math


class CmdVelToEcu(Node):
    def __init__(self):
        super().__init__("cmdvel_to_ecu")

        # Topics remain configurable so the command mux can be inserted without
        # changing the ECU message interface.
        self.ecu_pub = self.create_publisher(Ecu, "/ecu", 10)

        # 声明参数
        self.declare_parameter('L', 0.5)
        self.declare_parameter('MAX_STEER_DEG', 20.0)
        self.declare_parameter('LOW_SPEED_THRESH', 0.3)
        self.declare_parameter('STOP_THRESH', 0.02)
        self.declare_parameter('Vel_Offeset', 0.03)
        self.declare_parameter('OMEGA_DEADZONE', 0.20)
        self.declare_parameter('OMEGA_FULL_STEER', 1.20)
        self.declare_parameter('rear_wheel_flag', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('docking_mode', False)
        self.declare_parameter('docking_max_speed', 0.10)
        self.declare_parameter('docking_max_steer_deg', 8.0)
        self.declare_parameter('docking_stop_threshold', 0.001)
        self.declare_parameter('docking_ecu_speed_scale', 3.6)
        self.declare_parameter('input_timeout_sec', 0.20)

        # 读取参数
        self.L = self.get_parameter('L').value
        self.MAX_STEER_DEG = self.get_parameter('MAX_STEER_DEG').value
        self.LOW_SPEED_THRESH = self.get_parameter('LOW_SPEED_THRESH').value
        self.STOP_THRESH = self.get_parameter('STOP_THRESH').value
        self.Vel_Offeset = self.get_parameter('Vel_Offeset').value
        self.OMEGA_DEADZONE = self.get_parameter('OMEGA_DEADZONE').value
        self.OMEGA_FULL_STEER = self.get_parameter('OMEGA_FULL_STEER').value
        self.rear_wheel_flag = self.get_parameter('rear_wheel_flag').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.docking_mode = self.get_parameter('docking_mode').value
        self.docking_max_speed = abs(
            self.get_parameter('docking_max_speed').value)
        self.docking_max_steer_deg = abs(
            self.get_parameter('docking_max_steer_deg').value)
        self.docking_stop_threshold = abs(
            self.get_parameter('docking_stop_threshold').value)
        self.docking_ecu_speed_scale = self.get_parameter('docking_ecu_speed_scale').value
        if (not math.isfinite(self.docking_ecu_speed_scale)
                or self.docking_ecu_speed_scale <= 0):
            raise ValueError('docking ECU speed scale must be positive and finite')
        self.input_timeout_sec = max(
            0.05, self.get_parameter('input_timeout_sec').value)
        self.last_cmd_time = None
        self.last_command_stopped = False
        self.vehicle_stopped = False
        self.vehicle_time = None
        self.create_subscription(VehicleStatus, '/vehicle_status', self.vehicle_callback, 10)
        self.create_service(SetBool, '~/set_docking_mode', self.set_docking_mode)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        self.watchdog_timer = self.create_timer(
            min(0.05, self.input_timeout_sec / 2.0), self.watchdog_callback)
        self.get_logger().info(
            f"{self.cmd_vel_topic} → /ecu 节点已启动")

    def cmdvel_2_ecu(self, msg: Twist) -> Ecu:
        ecu_msg = Ecu()
        epsilon = 1e-6

        # 消息头
        ecu_msg.header = Header()
        ecu_msg.header.stamp = self.get_clock().now().to_msg()
        ecu_msg.header.frame_id = "base_link"

        # 获取速度与角速度
        v = msg.linear.x
        if not self.docking_mode and v >= self.STOP_THRESH:
            # Preserve the production navigation calibration. Historically the
            # positive-speed offset was not applied to reverse commands.
            v += self.Vel_Offeset
        omega = -msg.angular.z

        # ====================== 档位处理 ======================
        if v > 0:
            ecu_msg.shift = Ecu.SHIFT_D
        elif v < 0:
            ecu_msg.shift = Ecu.SHIFT_R
        else:
            ecu_msg.shift = Ecu.SHIFT_N

        # ====================== 速度限幅（保留正负号） ======================
        if self.docking_mode:
            # Docking must be allowed to command centimetre-per-second motion;
            # applying LOW_SPEED_THRESH here would turn 0.03 m/s into 0.8 m/s.
            v = max(-self.docking_max_speed, min(v, self.docking_max_speed))

        stop_threshold = self.docking_stop_threshold if self.docking_mode else self.STOP_THRESH
        if abs(v) <= stop_threshold:
            # 速度极小（接近停止指令），强制归零，不补偿死区
            v = 0.0
        elif not self.docking_mode and abs(v) <= self.LOW_SPEED_THRESH:
            v = self.LOW_SPEED_THRESH if v > 0 else -self.LOW_SPEED_THRESH

        # 设置电机输出
        # Twist is m/s; the current CAN setSpeed() encodes 0.1 km/h units.
        # Keep the existing production navigation calibration independent.
        ecu_msg.motor = abs(v) * (self.docking_ecu_speed_scale if self.docking_mode else 1.0)

        # ====================== 转向角计算 ======================
        if abs(v) <= epsilon:
            # 原地转向
            if abs(omega) < self.OMEGA_DEADZONE:
                steer_deg = 0.0
            else:
                steer_deg = omega * (
                    self.MAX_STEER_DEG / self.OMEGA_FULL_STEER)
        else:
            # 行驶转向（阿克曼）
            steer_deg = math.degrees(math.atan(self.L * omega / v))

        # 转向角限幅. Docking uses a smaller independent envelope.
        steer_limit = (self.docking_max_steer_deg
                       if self.docking_mode else self.MAX_STEER_DEG)
        ecu_msg.steer = max(-steer_limit, min(steer_deg, steer_limit))

        # ====================== 刹车与标志位 ======================
        ecu_msg.brake = abs(v) < epsilon
        ecu_msg.rear_wheel_flag = self.rear_wheel_flag

        return ecu_msg

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.last_command_stopped = (
            math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)
            and abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) < 1e-6)
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            msg = Twist()
        ecu_msg = self.cmdvel_2_ecu(msg)
        self.ecu_pub.publish(ecu_msg)

    def vehicle_callback(self, msg):
        self.vehicle_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.vehicle_stopped = math.isfinite(msg.cur_speed) and abs(msg.cur_speed / 3.6) <= 0.01

    def set_docking_mode(self, request, response):
        now = self.get_clock().now().nanoseconds / 1e9
        fresh = self.vehicle_time is not None and 0 <= now - self.vehicle_time <= self.input_timeout_sec
        command_fresh = (self.last_cmd_time is not None and
                         0 <= now - self.last_cmd_time.nanoseconds / 1e9 <= self.input_timeout_sec)
        response.success = bool(fresh and command_fresh and self.vehicle_stopped
                                and self.last_command_stopped)
        if response.success:
            self.docking_mode = request.data
            self.ecu_pub.publish(self.cmdvel_2_ecu(Twist()))
        response.message = 'mode switched while stopped' if response.success else 'measured and commanded stop required'
        return response

    def watchdog_callback(self):
        now = self.get_clock().now()
        if (self.last_cmd_time is None or
                (now - self.last_cmd_time).nanoseconds < 0 or
                (now - self.last_cmd_time).nanoseconds / 1e9 >
                self.input_timeout_sec):
            self.ecu_pub.publish(self.cmdvel_2_ecu(Twist()))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToEcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
