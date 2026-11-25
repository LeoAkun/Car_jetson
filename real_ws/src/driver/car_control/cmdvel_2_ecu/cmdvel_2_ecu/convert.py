import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yunle_msgs.msg import Ecu, VehicleStatus
from std_msgs.msg import Header
from std_srvs.srv import SetBool
import math


MPS_TO_KMH = 3.6


class CmdVelToEcu(Node):
    def __init__(self):
        super().__init__("cmdvel_to_ecu")

        # Topics remain configurable so the command mux can be inserted without
        # changing the ECU message interface.
        self.ecu_pub = self.create_publisher(Ecu, "/ecu", 10)

        # 声明参数
        self.declare_parameter('L', 0.5)
        # L=0.5m and 26.565deg correspond to a conservative 1.0m bicycle-model
        # turning radius, matching Smac and MPPI. The CAN layer hard-clamps near 27deg.
        self.declare_parameter('MAX_STEER_DEG', 26.56505117707799)
        # All linear-speed parameters are m/s. Historical ECU calibration was
        # in km/h; migrate its physical values rather than multiplying them.
        self.declare_parameter('navigation_max_speed', 2.0 / MPS_TO_KMH)
        self.declare_parameter('LOW_SPEED_THRESH', 0.3 / MPS_TO_KMH)
        self.declare_parameter('STOP_THRESH', 0.02 / MPS_TO_KMH)
        self.declare_parameter('Vel_Offeset', 0.03 / MPS_TO_KMH)
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
        self.declare_parameter('brake_on_zero_command', True)

        # 读取参数
        self.L = self.get_parameter('L').value
        self.MAX_STEER_DEG = self.get_parameter('MAX_STEER_DEG').value
        self.navigation_max_speed = self.get_parameter('navigation_max_speed').value
        self.LOW_SPEED_THRESH = self.get_parameter('LOW_SPEED_THRESH').value
        self.STOP_THRESH = self.get_parameter('STOP_THRESH').value
        self.Vel_Offeset = self.get_parameter('Vel_Offeset').value
        for name in ('navigation_max_speed', 'LOW_SPEED_THRESH',
                     'STOP_THRESH', 'Vel_Offeset'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and nonnegative (m/s)')
        if (self.navigation_max_speed <= 0.0
                or self.LOW_SPEED_THRESH > self.navigation_max_speed
                or self.STOP_THRESH >= self.navigation_max_speed):
            raise ValueError('navigation speed thresholds must fit within its m/s limit')
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
        self.brake_on_zero_command = bool(
            self.get_parameter('brake_on_zero_command').value)
        self.last_cmd_time = None
        self.vehicle_status_received_time = None
        self.autodrive_enabled = False
        self.output_active = False
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

        # Twist stays in SI units through limits and inverse kinematics.
        v_cmd = msg.linear.x
        omega = -msg.angular.z
        finite_input = math.isfinite(v_cmd) and math.isfinite(omega)
        if not finite_input:
            v_cmd = omega = 0.0
        speed_limit = (self.docking_max_speed if self.docking_mode
                       else self.navigation_max_speed)
        v_cmd = max(-speed_limit, min(v_cmd, speed_limit))
        stop_threshold = (self.docking_stop_threshold if self.docking_mode
                          else self.STOP_THRESH)
        # Check the requested speed before applying any forward compensation;
        # both signs at the stop boundary must remain stopped.
        if abs(v_cmd) <= stop_threshold:
            v_cmd = 0.0

        v_motor = v_cmd
        if not self.docking_mode and v_motor != 0.0:
            if v_motor > 0.0:
                v_motor += self.Vel_Offeset
            v_motor = math.copysign(
                max(abs(v_motor), self.LOW_SPEED_THRESH), v_motor)
            # Recovery/teleop can bypass Nav2 limits. Bound the final motor
            # request too, including the offset and low-speed compensation.
            v_motor = max(-speed_limit, min(v_motor, speed_limit))

        # ====================== 档位处理 ======================
        if v_motor > epsilon:
            ecu_msg.shift = Ecu.SHIFT_D
        elif v_motor < -epsilon:
            ecu_msg.shift = Ecu.SHIFT_R
        else:
            ecu_msg.shift = Ecu.SHIFT_N

        # ECU motor is km/h; CAN setSpeed() then quantizes it in 0.1 km/h.
        speed_scale = (self.docking_ecu_speed_scale if self.docking_mode
                       else MPS_TO_KMH)
        ecu_msg.motor = abs(v_motor) * speed_scale

        # ====================== 转向角计算 ======================
        if abs(v_motor) <= epsilon:
            # 原地转向
            if abs(omega) < self.OMEGA_DEADZONE:
                steer_deg = 0.0
            else:
                steer_deg = omega * (
                    self.MAX_STEER_DEG / self.OMEGA_FULL_STEER)
        else:
            # Use the speed that is actually sent to the motor. Otherwise the
            # low-speed floor raises yaw rate above the controller request.
            steer_deg = math.degrees(math.atan(self.L * omega / v_motor))

        # 转向角限幅. Docking uses a smaller independent envelope.
        steer_limit = (self.docking_max_steer_deg
                       if self.docking_mode else self.MAX_STEER_DEG)
        ecu_msg.steer = max(-steer_limit, min(steer_deg, steer_limit))

        # Restore the historical JD03 stop behavior only for a fresh, finite
        # zero-speed command. Timeout/invalid/manual-takeover release frames
        # explicitly clear this bit in _publish_neutral_once().
        ecu_msg.brake = bool(
            finite_input and self.brake_on_zero_command
            and abs(v_motor) < epsilon)
        ecu_msg.rear_wheel_flag = self.rear_wheel_flag

        return ecu_msg

    def cmd_callback(self, msg: Twist):
        now = self.get_clock().now()
        self.last_cmd_time = now
        finite = all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z))
        self.last_command_stopped = (finite
                                     and abs(msg.linear.x) < 1e-6
                                     and abs(msg.angular.z) < 1e-6)
        if not self._autodrive_feedback_fresh(now):
            if self.output_active and self.autodrive_enabled:
                self._publish_neutral_once()
            else:
                self.output_active = False
            return
        if not finite:
            self._publish_neutral_once()
            return
        ecu_msg = self.cmdvel_2_ecu(msg)
        self.ecu_pub.publish(ecu_msg)
        self.output_active = True

    def vehicle_callback(self, msg):
        self.vehicle_status_received_time = self.get_clock().now()
        self.autodrive_enabled = bool(msg.is_autodrive)
        if not self.autodrive_enabled:
            # The physical manual takeover has priority. Do not emit even a
            # zero ECU frame: every JD03 ECU frame requests AUTO_MODE.
            self.output_active = False
        self.vehicle_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.vehicle_stopped = math.isfinite(msg.cur_speed) and abs(msg.cur_speed / 3.6) <= 0.01

    def _autodrive_feedback_fresh(self, now):
        if not self.autodrive_enabled or self.vehicle_status_received_time is None:
            return False
        age = (now.nanoseconds - self.vehicle_status_received_time.nanoseconds) / 1e9
        return 0.0 <= age <= self.input_timeout_sec

    def _publish_neutral_once(self):
        if not self.output_active:
            return
        neutral_release = self.cmdvel_2_ecu(Twist())
        # JD03 maps the same flag to brake mode and EBrake. A watchdog release
        # must not leave the vehicle parked after navigation stops publishing.
        neutral_release.brake = False
        self.ecu_pub.publish(neutral_release)
        self.output_active = False

    def set_docking_mode(self, request, response):
        now_time = self.get_clock().now()
        now = now_time.nanoseconds / 1e9
        fresh = (self._autodrive_feedback_fresh(now_time)
                 and self.vehicle_time is not None
                 and 0 <= now - self.vehicle_time <= self.input_timeout_sec)
        command_fresh = (self.last_cmd_time is not None and
                         0 <= now - self.last_cmd_time.nanoseconds / 1e9 <= self.input_timeout_sec)
        response.success = bool(fresh and command_fresh and self.vehicle_stopped
                                and self.last_command_stopped)
        if response.success:
            self.docking_mode = request.data
            self.ecu_pub.publish(self.cmdvel_2_ecu(Twist()))
            self.output_active = True
        response.message = (
            'mode switched while stopped' if response.success
            else 'measured and commanded stop required')
        return response

    def watchdog_callback(self):
        if not self.output_active:
            return
        now = self.get_clock().now()
        if not self._autodrive_feedback_fresh(now):
            if self.autodrive_enabled:
                self._publish_neutral_once()
            else:
                self.output_active = False
            return
        command_age = ((now.nanoseconds - self.last_cmd_time.nanoseconds) / 1e9
                       if self.last_cmd_time is not None else math.inf)
        if command_age < 0.0 or command_age > self.input_timeout_sec:
            self._publish_neutral_once()


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
