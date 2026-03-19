#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_mock_publisher')
        # 发布到官方常用的 /fix 话题
        self.publisher_ = self.create_publisher(NavSatFix, '/sensing/gnss/pose_with_covariance', 10)
        
        # 每秒发布一次 (1Hz)
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        self.get_logger().info('GPS (NavSatFix) 模拟发布器已启动...')

    def publish_gps_data(self):
        msg = NavSatFix()

        # 1. 设置 Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link' # 通常 GPS 数据关联到 gps_link 坐标系

        # 2. 设置状态 (Status)
        # STATUS_FIX 表示已获取有效定位
        msg.status.status = NavSatStatus.STATUS_FIX
        # SERVICE_GPS 表示数据来源于 GPS
        msg.status.service = NavSatStatus.SERVICE_GPS

        # 3. 设置坐标信息 (基于您 Waypoint 消息中的定义字段)
        # 这里的数值可以根据您的测试需求修改
        msg.longitude = 7.0  # 经度 (Longitude) 
        msg.latitude = 1.0    # 纬度 (Latitude) 

        msg.altitude = 10.0       # 海拔高度

        # 4. 设置位置协方差 (Position Covariance)
        # 如果不确定具体精度，可以填入一个对角阵
        # 这里表示经纬高方向上的误差约为 0.1m
        msg.position_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        # 定义协方差类型为“已知对角线元素”
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布 GPS 坐标:Lng={msg.longitude} , Lat={msg.latitude} ')

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()