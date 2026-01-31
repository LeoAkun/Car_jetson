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
import requests
import time
from sensor_msgs.msg import NavSatFix, BatteryState
from yunle_msgs.msg import Battery, VehicleStatus
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Int32
from multi_map_navigation_msgs.msg import RobotStatus
from multi_map_navigation.process_manager import ProcessManager

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
        self.declare_parameter('remote_driving_url', 'https://example.com/api/remote-driving')  # 远程驾驶请求URL
        self.declare_parameter('speed_threshold_time', 300)  # 速度为0的持续时间阈值（秒），默认5分钟

        # 获取参数
        self.broker_url = self.get_parameter('broker_url').value
        self.port = self.get_parameter('port').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.client_id = self.get_parameter('client_id').value
        self.vin = self.get_parameter('vin').value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self.qos = self.get_parameter('qos').value
        self.remote_driving_url = self.get_parameter('remote_driving_url').value
        self.speed_threshold_time = self.get_parameter('speed_threshold_time').value

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

        # 车辆问题检测相关变量
        self.zero_speed_start_time = None  # 速度为0的开始时间
        self.last_remote_driving_request_time = 0  # 上次远程驾驶请求时间
        self.remote_driving_request_sent = False  # 是否已发送远程驾驶请求
        self.last_speed = None  # 上次速度值，用于检测变化
        self.last_log_time = 0  # 上次打印日志的时间，防止日志刷屏
        self.log_debounce_time = 2.0  # 防抖时间：2秒内不重复打印相同类型的日志

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

        # 订阅机器人任务状态 - 接收String类型
        self.robot_state_sub = self.create_subscription(
            String,
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

        # 创建问题检测定时器 - 每10秒检查一次
        self.problem_check_timer = self.create_timer(
            10.0,
            self.check_vehicle_problem
        )

        # 创建诊断定时器 - 每30秒打印一次诊断信息
        # self.diagnostic_timer = self.create_timer(
        #     30.0,
        #     self.print_diagnostics
        # )

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

        # 检测速度是否变化（避免重复处理）
        if self.last_speed is not None and abs(msg.cur_speed - self.last_speed) < 0.01:
            # 速度没有变化，不处理
            return

        # 速度确实变化了，记录并处理
        current_time = time.time()
        self.get_logger().debug(f'速度变化: {self.last_speed} -> {msg.cur_speed}')
        self.last_speed = msg.cur_speed

        # 检测速度是否为0
        is_zero_speed = (msg.cur_speed == 0.0)

        if is_zero_speed:
            # 速度为0
            if self.zero_speed_start_time is None:
                # 从非0变为0，开始计时
                self.zero_speed_start_time = current_time
                # 防抖：避免频繁打印
                if current_time - self.last_log_time > self.log_debounce_time:
                    self.get_logger().info('车辆速度为0，开始计时...')
                    self.last_log_time = current_time
                self.remote_driving_request_sent = False  # 重置请求标志
        else:
            # 速度不为0
            if self.zero_speed_start_time is not None:
                # 从0变为非0，重置计时器
                # 防抖：避免频繁打印
                if current_time - self.last_log_time > self.log_debounce_time:
                    self.get_logger().info(f'车辆恢复运动，速度: {msg.cur_speed:.1f} km/h')
                    self.last_log_time = current_time
                self.zero_speed_start_time = None
                self.remote_driving_request_sent = False

    def robot_state_callback(self, msg):
        """机器人任务状态回调 - 接收String类型，转换为int32存储"""
        # msg.data 是字符串: "idle" 或 "running"
        status_map = {"idle": 0, "running": 1}
        status_value = status_map.get(msg.data.lower())

        if status_value is not None:
            self.robot_status['task_status'] = status_value
        else:
            self.get_logger().warn(f'未知的任务状态: {msg.data}')

    def print_diagnostics(self):
        """打印诊断信息，帮助排查问题"""
        self.get_logger().info('='*60)
        self.get_logger().info('诊断信息:')
        self.get_logger().info(f'  当前速度: {self.robot_status.get("cur_speed", "N/A")} km/h')
        self.get_logger().info(f'  上次记录速度: {self.last_speed} km/h')
        self.get_logger().info(f'  任务状态: {"running" if self.robot_status.get("task_status") == 1 else "idle"}')
        if self.zero_speed_start_time is not None:
            elapsed = time.time() - self.zero_speed_start_time
            self.get_logger().info(f'  速度为0持续时间: {elapsed:.1f}秒 / {self.speed_threshold_time}秒')
        else:
            self.get_logger().info('  速度为0持续时间: 0秒 (未开始计时)')
        self.get_logger().info('='*60)

    def check_vehicle_problem(self):
        """检测车辆是否发生问题"""
        # 检查条件：速度为0且任务状态为running
        if self.zero_speed_start_time is not None:
            zero_speed_duration = time.time() - self.zero_speed_start_time
            task_is_running = self.robot_status['task_status'] == 1

            # 如果速度为0的时长超过阈值，且任务正在运行，且未发送过请求
            if (zero_speed_duration >= self.speed_threshold_time and
                task_is_running and
                not self.remote_driving_request_sent):

                self.get_logger().warn(
                    f'检测到车辆问题！速度为0已超过{self.speed_threshold_time}秒，'
                    f'任务状态为running，准备发送远程驾驶请求...'
                )

                # 发送远程驾驶请求
                self.send_remote_driving_request()
                self.remote_driving_request_sent = True

    def send_remote_driving_request(self):
        """发送HTTPS远程驾驶请求"""
        try:
            # 准备请求数据
            payload = {
                "vin": self.vin
            }

            self.get_logger().info(f'发送远程驾驶请求到: {self.remote_driving_url}')
            self.get_logger().info(f'请求数据: {json.dumps(payload)}')

            # 发送HTTPS POST请求
            response = requests.post(
                self.remote_driving_url,
                json=payload,
                timeout=10,
                verify=True  # 验证SSL证书
            )

            # 检查响应
            if response.status_code == 200:
                self.get_logger().info(f'远程驾驶请求发送成功！响应: {response.text}')

                # 请求成功后，关闭所有自动驾驶程序
                self.get_logger().warn('远程驾驶请求已确认，正在关闭所有自动驾驶进程...')
                self.shutdown_all_auto_driving_processes()

            else:
                self.get_logger().error(
                    f'远程驾驶请求失败，状态码: {response.status_code}, '
                    f'响应: {response.text}'
                )

        except requests.exceptions.Timeout:
            self.get_logger().error('远程驾驶请求超时')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'远程驾驶请求异常: {e}')
        except Exception as e:
            self.get_logger().error(f'发送远程驾驶请求时出错: {e}')

    def shutdown_all_auto_driving_processes(self):
        """关闭所有自动驾驶进程"""
        try:
            # 创建进程管理器实例
            process_manager = ProcessManager()

            # 调用shutdown_all_processes方法
            success = process_manager.shutdown_all_processes()

            if success:
                self.get_logger().info('所有自动驾驶进程已成功关闭')
            else:
                self.get_logger().error('部分自动驾驶进程关闭失败')

        except Exception as e:
            self.get_logger().error(f'关闭自动驾驶进程时出错: {e}')

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

