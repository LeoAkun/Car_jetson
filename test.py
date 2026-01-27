import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
import math


class MockGpsTfProvider(Node):
    def __init__(self):
        super().__init__('mock_gps_tf_provider')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.gps_publisher = self.create_publisher(
            NavSatFix,
            '/sensing/gnss/pose_with_covariance',
            10
        )

        # 每 0.01 秒更新一次counter
        self.timer_counter = self.create_timer(0.01, self.update_counter)

        # 每 0.05 秒更新一次gps坐标
        self.timer_gps = self.create_timer(0.05, self.update_gps)

        # 每 0.03 秒更新一次tf
        self.timer_gps = self.create_timer(0.03, self.update_tf)

        self.counter = 0  # 从 0 开始计数
        self.get_logger().info("模拟节点启动：TF 和 GPS 的 x 坐标每秒 +1，循环 0~9")

    def update_counter(self):
        self.counter += 1  # 每 0.1 秒 +1

    def update_gps(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        sim_x = self.counter
        sim_y = 0.0
        # ────────────────────────────────
        # 发布 GPS：完全跟随 sim_x
        # ────────────────────────────────

        gps_msg = NavSatFix()
        gps_msg.header.stamp = now_msg
        gps_msg.header.frame_id = "gnss_link"

        gps_msg.latitude  = sim_y
        gps_msg.longitude = sim_x * 1.0

        gps_msg.altitude = 0.0

        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_msg.position_covariance = [0.0001, 0.0, 0.0,
                                       0.0, 0.0001, 0.0,
                                       0.0, 0.0, 0.1]

        self.gps_publisher.publish(gps_msg)

        # 每步都打印一次，便于观察
        self.get_logger().info(
            f"x = {sim_x:.0f} m | "
            f"GPS: ({gps_msg.latitude:.8f}, {gps_msg.longitude:.8f})"
        )
    
    def update_tf(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()
        
        sim_x = self.counter
        sim_y = 0.0

        # ────────────────────────────────
        # 发布 TF: map → base_link
        # ────────────────────────────────
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(sim_x)
        t.transform.translation.y = sim_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        

        # 每步都打印一次，便于观察
        self.get_logger().info(
            f"x = {sim_x:.0f} m | "
            f"tf: ({t.transform.translation.x:.8f}, {t.transform.translation.y:.8f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MockGpsTfProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()