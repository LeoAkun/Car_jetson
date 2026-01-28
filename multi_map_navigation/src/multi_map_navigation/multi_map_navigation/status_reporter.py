#!/usr/bin/env python3
"""
状态报告器节点
开发者A - 通信模块

从各种ROS2主题聚合机器人状态并发布到MQTT代理
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from sensor_msgs.msg import NavSatFix, BatteryState
from yunle_msgs.msg import Battery, VehicleStatus 
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Int32
from multi_map_navigation_msgs.msg import RobotStatus

# 导入yunle_msgs的VehicleStatus
try:
    from yunle_msgs.msg import VehicleStatus
    VEHICLE_STATUS_AVAILABLE = True
except ImportError:
    VEHICLE_STATUS_AVAILABLE = False


class StatusReporter(Node):
    def __init__(self):
        super().__init__('status_reporter')

        # 声明参数
        self.declare_parameter('broker_url', 'localhost')
        self.declare_parameter('port', 1883)
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('client_id', 'robot_status_001')
        self.declare_parameter('vin', 'LS1234567890')  # 修改：使用vin
        self.declare_parameter('heartbeat_rate', 3.0)
        self.declare_parameter('qos', 1)

        # 获取参数
        self.broker_url = self.get_parameter('broker_url').value
        self.port = self.get_parameter('port').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.client_id = self.get_parameter('client_id').value
        self.vin = self.get_parameter('vin').value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self.qos = self.get_parameter('qos').value

        # 构建MQTT topic - 格式: prod/data/vehicle/{vin}/vehicle_status
        self.status_topic = f'prod/data/vehicle/{self.vin}/vehicle_status'

        # 机器人状态数据
        self.robot_status = {
            'vin': self.vin,
            'task_status': 0,  # 修改：整数类型，0=idle, 1=running
            'gps_lng': 0.0,
            'gps_lat': 0.0,
            'gps_alt': 0.0,
            'cur_speed': 0.0,
            'battery_capacity': 0.0
        }

        # ROS2订阅器 - GPS位置信息
        self.gnss_pose_sub = self.create_subscription(
            NavSatFix,
            '/sensing/gnss/pose_with_covariance',
            self.gnss_pose_callback,
            10
        )

        # 订阅电池状态
        self.battery_sub = self.create_subscription(
            Battery,
            '/battery_status',
            self.battery_callback,
            10
        )

        # 订阅车辆状态
        if VEHICLE_STATUS_AVAILABLE:
            self.vehicle_status_sub = self.create_subscription(
                VehicleStatus,
                '/vehicle_status',
                self.vehicle_status_callback,
                10
            )
            self.get_logger().info('已订阅 /vehicle_status 话题')
        else:
            self.get_logger().warn('yunle_msgs.VehicleStatus 不可用，将无法获取车速信息')

        # 订阅机器人任务状态 - 修改：接收Int32类型
        self.robot_state_sub = self.create_subscription(
            Int32,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        # ROS2发布器 - 发布RobotStatus消息
        self.robot_status_pub = self.create_publisher(
            RobotStatus,
            '/robot_status',
            10
        )

        # MQTT客户端设置
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect

        # 如果提供了用户名和密码则设置
        if self.username and self.password:
            self.mqtt_client.username_pw_set(self.username, self.password)

        # 连接到代理
        self.connect_to_broker()

        # 在后台启动MQTT循环
        self.mqtt_client.loop_start()

        # 创建心跳定时器
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_rate,
            self.publish_heartbeat
        )

        self.get_logger().info(
            f'状态报告器已初始化\n'
            f'  VIN: {self.vin}\n'
            f'  MQTT Broker: {self.broker_url}:{self.port}\n'
            f'  MQTT Topic: {self.status_topic}\n'
            f'  心跳频率: {self.heartbeat_rate}秒'
        )

    def connect_to_broker(self):
        """连接到MQTT代理"""
        try:
            self.get_logger().info(f'正在连接到MQTT代理 {self.broker_url}:{self.port}')
            self.mqtt_client.connect(self.broker_url, self.port, keepalive=60)
        except Exception as e:
            self.get_logger().error(f'连接MQTT代理失败: {e}')

    def on_connect(self, client, userdata, flags, rc):
        """连接到MQTT代理时的回调"""
        if rc == 0:
            self.get_logger().info('成功连接到MQTT代理')
        else:
            self.get_logger().error(f'连接MQTT代理失败，返回码: {rc}')

    def on_disconnect(self, client, userdata, rc):
        """从MQTT代理断开连接时的回调"""
        if rc != 0:
            self.get_logger().warn(f'意外断开与MQTT代理的连接，返回码: {rc}')
            self.get_logger().info('正在尝试重新连接...')

    def gnss_pose_callback(self, msg):
        """GNSS位姿数据回调 - 从PoseWithCovarianceStamped提取GPS坐标"""
        # 注意: 这里假设position.x和position.y存储的是经纬度
        # 如果实际存储的是UTM坐标，需要进行坐标转换
        self.robot_status['gps_lng'] = msg.longitude
        self.robot_status['gps_lat'] = msg.latitude
        self.robot_status['gps_alt'] = msg.altitude

    def battery_callback(self, msg):
        """电池数据回调"""
        self.robot_status['battery_capacity'] = msg.capacity

    def vehicle_status_callback(self, msg):
        """车辆状态回调 - 获取当前速度"""
        self.robot_status['cur_speed'] = msg.cur_speed

    def robot_state_callback(self, msg):
        """机器人任务状态回调 - 接收Int32类型"""
        # msg.data 应该是整数: 0=idle, 1=running
        self.robot_status['task_status'] = msg.data

    def publish_heartbeat(self):
        """向MQTT代理发布心跳"""
        try:
            # 创建RobotStatus消息
            robot_status_msg = RobotStatus()
            robot_status_msg.header.stamp = self.get_clock().now().to_msg()
            robot_status_msg.header.frame_id = 'base_link'
            robot_status_msg.vin = self.robot_status['vin']  # 修改：使用vin
            robot_status_msg.task_status = self.robot_status['task_status']  # int32类型
            robot_status_msg.gps_lng = self.robot_status['gps_lng']
            robot_status_msg.gps_lat = self.robot_status['gps_lat']
            robot_status_msg.gps_alt = self.robot_status['gps_alt']
            robot_status_msg.cur_speed = self.robot_status['cur_speed']
            robot_status_msg.battery_capacity = self.robot_status['battery_capacity']

            # 发布ROS2消息
            self.robot_status_pub.publish(robot_status_msg)

            # 准备MQTT JSON数据 - 严格按照Schema定义
            mqtt_payload = {
                'vin': self.robot_status['vin'],
                'gps_lng': self.robot_status['gps_lng'],
                'gps_lat': self.robot_status['gps_lat'],
                'task_status': self.robot_status['task_status'],  # 整数类型
                'cur_speed': self.robot_status['cur_speed'],
                'battery_capacity': self.robot_status['battery_capacity']
            }

            # 转换为JSON
            payload = json.dumps(mqtt_payload)

            # 发布到MQTT
            result = self.mqtt_client.publish(
                self.status_topic,
                payload,
                qos=self.qos
            )

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                task_status_str = 'running' if mqtt_payload['task_status'] == 1 else 'idle'
                self.get_logger().debug(
                    f'已发布状态到 {self.status_topic}:\n'
                    f'  VIN: {mqtt_payload["vin"]}\n'
                    f'  任务状态: {task_status_str} ({mqtt_payload["task_status"]})\n'
                    f'  位置: ({mqtt_payload["gps_lng"]:.6f}, {mqtt_payload["gps_lat"]:.6f})\n'
                    f'  速度: {mqtt_payload["cur_speed"]:.2f} km/h\n'
                    f'  电量: {mqtt_payload["battery_capacity"]:.1f}%'
                )
            else:
                self.get_logger().warn(f'发布心跳失败，返回码: {result.rc}')

        except Exception as e:
            self.get_logger().error(f'发布心跳时出错: {e}')

    def destroy_node(self):
        """节点销毁时的清理"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusReporter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
# """
# 状态报告器节点
# 开发者A - 通信模块

# 从各种ROS2主题聚合机器人状态并发布到MQTT代理
# """

# import rclpy
# from rclpy.node import Node
# import paho.mqtt.client as mqtt
# import json
# import time
# from sensor_msgs.msg import NavSatFix
# from yunle_msgs.msg import Battery, VehicleStatus 
# from std_msgs.msg import Int32
# from multi_map_navigation_msgs.msg import RobotStatus

# # 导入yunle_msgs的VehicleStatus
# try:
#     from yunle_msgs.msg import VehicleStatus
#     VEHICLE_STATUS_AVAILABLE = True
# except ImportError:
#     VEHICLE_STATUS_AVAILABLE = False


# class StatusReporter(Node):
#     def __init__(self):
#         super().__init__('status_reporter')

#         # 声明参数
#         self.declare_parameter('broker_url', 'localhost')
#         self.declare_parameter('port', 1883)
#         self.declare_parameter('username', '')
#         self.declare_parameter('password', '')
#         self.declare_parameter('client_id', 'robot_status_001')
#         self.declare_parameter('vin', 'LS1234567890')
#         self.declare_parameter('heartbeat_rate', 3.0)
#         self.declare_parameter('qos', 1)
#         self.declare_parameter('data_timeout', 5.0)  # 新增：数据超时时间（秒）

#         # 获取参数
#         self.broker_url = self.get_parameter('broker_url').value
#         self.port = self.get_parameter('port').value
#         self.username = self.get_parameter('username').value
#         self.password = self.get_parameter('password').value
#         self.client_id = self.get_parameter('client_id').value
#         self.vin = self.get_parameter('vin').value
#         self.heartbeat_rate = self.get_parameter('heartbeat_rate').value
#         self.qos = self.get_parameter('qos').value
#         self.data_timeout = self.get_parameter('data_timeout').value

#         # 构建MQTT topic
#         self.status_topic = f'prod/data/vehicle/{self.vin}/vehicle_status'

#         # 机器人状态数据（默认值）
#         self.default_status = {
#             'vin': self.vin,
#             'task_status': 0,
#             'gps_lng': 0.0,
#             'gps_lat': 0.0,
#             'gps_alt': 0.0,
#             'cur_speed': 0.0,
#             'battery_capacity': 0.0
#         }
        
#         # 当前状态
#         self.robot_status = self.default_status.copy()

#         # 数据接收时间戳（用于超时检测）
#         self.last_update_time = {
#             'gps': 0.0,
#             'battery': 0.0,
#             'speed': 0.0,
#             'task_status': 0.0
#         }

#         # ROS2订阅器 - GPS位置信息
#         self.gnss_pose_sub = self.create_subscription(
#             NavSatFix,
#             '/sensing/gnss/pose_with_covariance',
#             self.gnss_pose_callback,
#             10
#         )
#         self.get_logger().info('已订阅 /sensing/gnss/pose_with_covariance (NavSatFix)')

#         # 订阅电池状态
#         self.battery_sub = self.create_subscription(
#             Battery,
#             '/battery_status',
#             self.battery_callback,
#             10
#         )
#         self.get_logger().info('已订阅 /battery_status (yunle_msgs/Battery)')

#         # 订阅车辆状态
#         if VEHICLE_STATUS_AVAILABLE:
#             self.vehicle_status_sub = self.create_subscription(
#                 VehicleStatus,
#                 '/vehicle_status',
#                 self.vehicle_status_callback,
#                 10
#             )
#             self.get_logger().info('已订阅 /vehicle_status (yunle_msgs/VehicleStatus)')
#         else:
#             self.get_logger().warn('yunle_msgs.VehicleStatus 不可用，将无法获取车速信息')

#         # 订阅机器人任务状态
#         self.robot_state_sub = self.create_subscription(
#             Int32,
#             '/robot_state',
#             self.robot_state_callback,
#             10
#         )
#         self.get_logger().info('已订阅 /robot_state (Int32)')

#         # ROS2发布器
#         self.robot_status_pub = self.create_publisher(
#             RobotStatus,
#             '/robot_status',
#             10
#         )

#         # MQTT客户端设置
#         self.mqtt_client = mqtt.Client(client_id=self.client_id)
#         self.mqtt_client.on_connect = self.on_connect
#         self.mqtt_client.on_disconnect = self.on_disconnect

#         if self.username and self.password:
#             self.mqtt_client.username_pw_set(self.username, self.password)

#         self.connect_to_broker()
#         self.mqtt_client.loop_start()

#         # 创建心跳定时器
#         self.heartbeat_timer = self.create_timer(
#             self.heartbeat_rate,
#             self.publish_heartbeat
#         )

#         self.get_logger().info(
#             f'状态报告器已初始化\n'
#             f'  VIN: {self.vin}\n'
#             f'  MQTT Broker: {self.broker_url}:{self.port}\n'
#             f'  MQTT Topic: {self.status_topic}\n'
#             f'  心跳频率: {self.heartbeat_rate}秒\n'
#             f'  数据超时: {self.data_timeout}秒'
#         )

#     def connect_to_broker(self):
#         """连接到MQTT代理"""
#         try:
#             self.get_logger().info(f'正在连接到MQTT代理 {self.broker_url}:{self.port}')
#             self.mqtt_client.connect(self.broker_url, self.port, keepalive=60)
#         except Exception as e:
#             self.get_logger().error(f'连接MQTT代理失败: {e}')

#     def on_connect(self, client, userdata, flags, rc):
#         """连接到MQTT代理时的回调"""
#         if rc == 0:
#             self.get_logger().info('✓ 成功连接到MQTT代理')
#         else:
#             self.get_logger().error(f'✗ 连接MQTT代理失败，返回码: {rc}')

#     def on_disconnect(self, client, userdata, rc):
#         """从MQTT代理断开连接时的回调"""
#         if rc != 0:
#             self.get_logger().warn(f'⚠ 意外断开与MQTT代理的连接，返回码: {rc}')
#             self.get_logger().info('正在尝试重新连接...')

#     def gnss_pose_callback(self, msg):
#         """GNSS位姿数据回调"""
#         self.robot_status['gps_lng'] = msg.longitude
#         self.robot_status['gps_lat'] = msg.latitude
#         self.robot_status['gps_alt'] = msg.altitude
#         self.last_update_time['gps'] = time.time()  # 记录更新时间
        
#         self.get_logger().info(
#             f'📍 GPS更新: ({msg.longitude:.6f}, {msg.latitude:.6f}, {msg.altitude:.1f}m)'
#         )

#     def battery_callback(self, msg):
#         """电池数据回调"""
#         self.robot_status['battery_capacity'] = msg.capacity
#         self.last_update_time['battery'] = time.time()  # 记录更新时间
        
#         charge_status = "充电中" if msg.charge_status == 1 else "放电中"
#         self.get_logger().info(
#             f'🔋 电池更新: {msg.capacity:.1f}% | {msg.voltage:.1f}V | {msg.ampere:.1f}A | {charge_status}'
#         )

#     def vehicle_status_callback(self, msg):
#         """车辆状态回调"""
#         self.robot_status['cur_speed'] = msg.cur_speed
#         self.last_update_time['speed'] = time.time()  # 记录更新时间
        
#         self.get_logger().info(f'🚗 速度更新: {msg.cur_speed:.1f} km/h')

#     def robot_state_callback(self, msg):
#         """机器人任务状态回调"""
#         self.robot_status['task_status'] = msg.data
#         self.last_update_time['task_status'] = time.time()  # 记录更新时间
        
#         status_str = 'running' if msg.data == 1 else 'idle'
#         self.get_logger().info(f'📋 任务状态更新: {status_str} ({msg.data})')

#     def check_data_timeout(self):
#         """检查数据是否超时，超时则恢复默认值"""
#         current_time = time.time()
#         timeout_occurred = False

#         # 检查GPS超时
#         if current_time - self.last_update_time['gps'] > self.data_timeout:
#             if self.robot_status['gps_lng'] != 0.0 or self.robot_status['gps_lat'] != 0.0:
#                 self.get_logger().warn('⚠ GPS数据超时，恢复默认值')
#                 self.robot_status['gps_lng'] = self.default_status['gps_lng']
#                 self.robot_status['gps_lat'] = self.default_status['gps_lat']
#                 self.robot_status['gps_alt'] = self.default_status['gps_alt']
#                 timeout_occurred = True

#         # 检查电池超时
#         if current_time - self.last_update_time['battery'] > self.data_timeout:
#             if self.robot_status['battery_capacity'] != 0.0:
#                 self.get_logger().warn('⚠ 电池数据超时，恢复默认值')
#                 self.robot_status['battery_capacity'] = self.default_status['battery_capacity']
#                 timeout_occurred = True

#         # 检查速度超时
#         if current_time - self.last_update_time['speed'] > self.data_timeout:
#             if self.robot_status['cur_speed'] != 0.0:
#                 self.get_logger().warn('⚠ 速度数据超时，恢复默认值')
#                 self.robot_status['cur_speed'] = self.default_status['cur_speed']
#                 timeout_occurred = True

#         # 检查任务状态超时
#         if current_time - self.last_update_time['task_status'] > self.data_timeout:
#             if self.robot_status['task_status'] != 0:
#                 self.get_logger().warn('⚠ 任务状态超时，恢复默认值')
#                 self.robot_status['task_status'] = self.default_status['task_status']
#                 timeout_occurred = True

#         return timeout_occurred

#     def publish_heartbeat(self):
#         """向MQTT代理发布心跳"""
#         try:
#             # 检查数据超时
#             self.check_data_timeout()

#             # 创建RobotStatus消息
#             robot_status_msg = RobotStatus()
#             robot_status_msg.header.stamp = self.get_clock().now().to_msg()
#             robot_status_msg.header.frame_id = 'base_link'
#             robot_status_msg.vin = self.robot_status['vin']
#             robot_status_msg.task_status = self.robot_status['task_status']
#             robot_status_msg.gps_lng = self.robot_status['gps_lng']
#             robot_status_msg.gps_lat = self.robot_status['gps_lat']
#             robot_status_msg.gps_alt = self.robot_status['gps_alt']
#             robot_status_msg.cur_speed = self.robot_status['cur_speed']
#             robot_status_msg.battery_capacity = self.robot_status['battery_capacity']

#             # 发布ROS2消息
#             self.robot_status_pub.publish(robot_status_msg)

#             # 准备MQTT JSON数据
#             mqtt_payload = {
#                 'vin': self.robot_status['vin'],
#                 'gps_lng': self.robot_status['gps_lng'],
#                 'gps_lat': self.robot_status['gps_lat'],
#                 'task_status': self.robot_status['task_status'],
#                 'cur_speed': self.robot_status['cur_speed'],
#                 'battery_capacity': self.robot_status['battery_capacity']
#             }

#             # 转换为JSON
#             payload = json.dumps(mqtt_payload)

#             # 发布到MQTT
#             result = self.mqtt_client.publish(
#                 self.status_topic,
#                 payload,
#                 qos=self.qos
#             )

#             if result.rc == mqtt.MQTT_ERR_SUCCESS:
#                 task_status_str = 'running' if mqtt_payload['task_status'] == 1 else 'idle'
#                 self.get_logger().info(
#                     f'✓ 状态已发布 | '
#                     f'任务:{task_status_str} | '
#                     f'速度:{mqtt_payload["cur_speed"]:.1f}km/h | '
#                     f'电量:{mqtt_payload["battery_capacity"]:.0f}% | '
#                     f'位置:({mqtt_payload["gps_lng"]:.6f},{mqtt_payload["gps_lat"]:.6f})'
#                 )
#             else:
#                 self.get_logger().warn(f'✗ 发布失败，返回码: {result.rc}')

#         except Exception as e:
#             self.get_logger().error(f'✗ 发布心跳时出错: {e}')
#             import traceback
#             self.get_logger().error(traceback.format_exc())

#     def destroy_node(self):
#         """节点销毁时的清理"""
#         self.mqtt_client.loop_stop()
#         self.mqtt_client.disconnect()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = StatusReporter()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
