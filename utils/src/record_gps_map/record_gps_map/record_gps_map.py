import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import NavSatFix
import csv
import os


class GpsMapLogger(Node):
    def __init__(self):
        super().__init__('gps_map_logger')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅 GPS（只用来保持最新数据）
        self.subscription = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/pose_with_covariance',
            self.gps_callback,
            10
        )

        # 每 2 秒记录一次
        self.timer = self.create_timer(2.0, self.record_callback)

        # 保存最新的 GPS 消息
        self.latest_gps = None

        # CSV 文件路径
        self.csv_file = '/home/akun/workspace/Car_jetson/utils/src/record_gps_map/record_gps_map.csv'

        # 如果文件存在，先删除它
        if os.path.exists(self.csv_file):
            try:
                os.remove(self.csv_file)
                self.get_logger().info(f"已删除旧的 CSV 文件: {self.csv_file}")
            except Exception as e:
                self.get_logger().warn(f"删除旧 CSV 文件失败: {e}")

        # 无论是否存在，都重新创建一个空的 CSV 文件
        with open(self.csv_file, 'w', newline='') as f:
            pass  # 创建空文件（或在这里写入表头）
        # 如果文件不存在，创建空文件

        self.get_logger().info('节点已启动：每 2 秒记录一次 GPS ↔ map 坐标')
        self.get_logger().info(f'保存路径: {os.path.abspath(self.csv_file)}')
        self.get_logger().info('CSV 格式: latitude,longitude,map_x,map_y')

    def gps_callback(self, msg: NavSatFix):
        """只保留最新的 GPS 数据"""
        self.latest_gps = msg

    def record_callback(self):
        if self.latest_gps is None:
            return

        try:
            gps_time = self.latest_gps.header.stamp

            t = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='base_link',
                time=gps_time,                      # 使用 GPS 的时间戳
                timeout=Duration(seconds=0.5)       # 加大到 0.5 秒，容忍常见延迟
            )

            # 成功获取后，检查实际使用的 TF 时间（可选，调试用）
            tf_time_sec = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            gps_time_sec = gps_time.sec + gps_time.nanosec * 1e-9
            diff_ms = (gps_time_sec - tf_time_sec) * 1000

            map_x = t.transform.translation.x
            map_y = t.transform.translation.y
            lat = self.latest_gps.latitude
            lon = self.latest_gps.longitude

            self.save_to_csv(lat, lon, map_x, map_y)

            print(f"[时间戳(非小数点):{gps_time.sec}, {gps_time.nanosec}]  "
                  f"[时间戳(小数点):{gps_time_sec}]"
                f"GPS: ({lat:.8f}, {lon:.8f})  →  "
                f"Map: x={map_x:.3f}, y={map_y:.3f}    "
                f"[时间差 {diff_ms:+.1f} ms] 已保存")

        except ExtrapolationException:
            self.get_logger().debug_throttle(5.0, "GPS 时间在tf之后，跳过本次记录")
        except (LookupException, ConnectivityException):
            pass
        except Exception as e:
            self.get_logger().error(f"记录异常: {e}")

    def save_to_csv(self, lat: float, lon: float, map_x: float, map_y: float):
        """追加写入一行数据"""
        row = [f"{lat:.8f}", f"{lon:.8f}", f"{map_x:.3f}", f"{map_y:.3f}"]

        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f, quoting=csv.QUOTE_MINIMAL)
            writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = GpsMapLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()