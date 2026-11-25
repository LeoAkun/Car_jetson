# #!/usr/bin/env python3
# """
# MQTT任务接收器节点
# 开发者A - 通信模块

# 订阅MQTT代理以接收TaskGraph任务并发布StartEndGraph到ROS2
# """

# import rclpy
# from rclpy.node import Node
# import paho.mqtt.client as mqtt
# import json
# from std_msgs.msg import Header
# from multi_map_navigation_msgs.msg import Waypoint, StartEndGraph
# from multi_map_navigation_msgs.msg import Edge as EdgeMsg


# class MQTTTaskReceiver(Node):
#     def __init__(self):
#         super().__init__('mqtt_task_receiver')

#         # 声明参数
#         self.declare_parameter('broker_url', 'localhost')
#         self.declare_parameter('port', 1883)
#         self.declare_parameter('username', '')
#         self.declare_parameter('password', '')
#         self.declare_parameter('client_id', 'robot_nav_001')
#         self.declare_parameter('task_topic', 'robot/task')
#         self.declare_parameter('qos', 1)
#         self.declare_parameter('reconnect_delay', 5.0)

#         # 获取参数
#         self.broker_url = self.get_parameter('broker_url').value
#         self.port = self.get_parameter('port').value
#         self.username = self.get_parameter('username').value
#         self.password = self.get_parameter('password').value
#         self.client_id = self.get_parameter('client_id').value
#         self.task_topic = self.get_parameter('task_topic').value
#         self.qos = self.get_parameter('qos').value
#         self.reconnect_delay = self.get_parameter('reconnect_delay').value

#         # 连接状态
#         self.is_connected = False

#         # ROS2发布器
#         self.graph_pub = self.create_publisher(
#             StartEndGraph,
#             '/start_end_graph',
#             10
#         )

#         # MQTT客户端设置
#         self.mqtt_client = mqtt.Client(client_id=self.client_id)
#         self.mqtt_client.on_connect = self.on_connect
#         self.mqtt_client.on_message = self.on_message
#         self.mqtt_client.on_disconnect = self.on_disconnect

#         # 如果提供了用户名和密码则设置
#         if self.username and self.password:
#             self.mqtt_client.username_pw_set(self.username, self.password)

#         # 连接到代理
#         self.connect_to_broker()

#         # 在后台启动MQTT循环
#         self.mqtt_client.loop_start()

#         # 创建定时器检查连接状态
#         self.reconnect_timer = self.create_timer(
#             self.reconnect_delay,
#             self.check_connection
#         )

#         self.get_logger().info('MQTT任务接收器已初始化')

#     def connect_to_broker(self):
#         """连接到MQTT代理"""
#         try:
#             self.get_logger().info(f'正在连接到MQTT代理 {self.broker_url}:{self.port}')
#             self.mqtt_client.connect(self.broker_url, self.port, keepalive=60)
#         except Exception as e:
#             self.get_logger().error(f'连接MQTT代理失败: {e}')
#             self.get_logger().info(f'将在 {self.reconnect_delay} 秒后重试...')

#     def check_connection(self):
#         """定期检查连接状态并尝试重连"""
#         if not self.is_connected:
#             self.get_logger().warn('未连接到MQTT代理，尝试重新连接...')
#             try:
#                 self.mqtt_client.reconnect()
#             except Exception as e:
#                 self.get_logger().debug(f'重连失败: {e}')

#     def on_connect(self, client, userdata, flags, rc):
#         """连接到MQTT代理时的回调"""
#         if rc == 0:
#             self.is_connected = True
#             self.get_logger().info('✅ 成功连接到MQTT代理')
#             # 订阅任务主题
#             client.subscribe(self.task_topic, qos=self.qos)
#             self.get_logger().info(f'📡 已订阅主题: {self.task_topic}')
#         else:
#             self.is_connected = False
#             error_messages = {
#                 1: '连接被拒绝 - 协议版本不正确',
#                 2: '连接被拒绝 - 客户端ID无效',
#                 3: '连接被拒绝 - 服务器不可用',
#                 4: '连接被拒绝 - 用户名或密码错误',
#                 5: '连接被拒绝 - 未授权'
#             }
#             error_msg = error_messages.get(rc, f'未知错误，返回码: {rc}')
#             self.get_logger().error(f'❌ 连接MQTT代理失败: {error_msg}')

#     def on_disconnect(self, client, userdata, rc):
#         """从MQTT代理断开连接时的回调"""
#         self.is_connected = False
#         if rc != 0:
#             self.get_logger().warn(f'⚠️  意外断开与MQTT代理的连接，返回码: {rc}')
#             self.get_logger().info('🔄 将自动尝试重新连接...')
#         else:
#             self.get_logger().info('已正常断开MQTT连接')

#     def on_message(self, client, userdata, msg):
#         """从MQTT代理接收消息时的回调"""
#         try:
#             self.get_logger().info(f'📨 收到消息，主题: {msg.topic}')
            
#             # 解析 JSON 字符串为列表
#             waypoints_data = json.loads(msg.payload.decode('utf-8'))
            
#             # 验证数据格式
#             if not isinstance(waypoints_data, list):
#                 self.get_logger().error(f'❌ 数据格式错误: 期望列表，收到 {type(waypoints_data).__name__}')
#                 return
            
#             self.get_logger().info(f'📋 收到 {len(waypoints_data)} 个航点')

#             # 解析航点列表
#             waypoint_list = self.parse_waypoint_list(waypoints_data)

#             # 发布到ROS2
#             self.publish_to_ros(waypoint_list)

#         except json.JSONDecodeError as e:
#             self.get_logger().error(f'❌ JSON解析失败: {e}')
#             self.get_logger().error(f'原始消息: {msg.payload.decode("utf-8", errors="replace")[:200]}...')
#         except Exception as e:
#             self.get_logger().error(f'❌ 处理消息时出错: {e}')
#             import traceback
#             self.get_logger().error(traceback.format_exc())

#     def parse_waypoint_list(self, waypoints_data):
#         """
#         将航点数据列表解析为WaypointList消息

#         输入格式:
#         [
#             {
#                 "id": 1,
#                 "map_id": 1,
#                 "map_name": "测试地图",
#                 "name": "A",
#                 "gps_lat": 30.762005,
#                 "gps_lng": 103.981704,
#                 "gps_heading": 0,
#                 "slam_x": 0,
#                 "slam_y": 0,
#                 "slam_yaw": 0,
#                 "type": 1,
#                 "created_at": "2026-01-28T12:40:11",
#                 "selected": false
#             },
#             ...
#         ]
#         """
#         waypoint_list_msg = WaypointList()
#         waypoint_list_msg.header = Header()
#         waypoint_list_msg.header.stamp = self.get_clock().now().to_msg()
#         waypoint_list_msg.header.frame_id = 'map'
        
#         # 生成任务ID
#         waypoint_list_msg.task_id = f'task_{self.get_clock().now().to_msg().sec}'
#         waypoint_list_msg.total_waypoints = len(waypoints_data)

#         self.get_logger().info(f'📊 开始解析 {waypoint_list_msg.total_waypoints} 个航点...')

#         # 解析每个航点
#         for idx, wp_data in enumerate(waypoints_data):
#             try:
#                 waypoint = Waypoint()

#                 # 基本信息
#                 waypoint.name = str(wp_data.get('name', ''))
#                 waypoint.id = int(wp_data.get('map_id', 0))
#                 waypoint.map_name = str(wp_data.get('map_name', ''))

#                 # GPS坐标
#                 waypoint.lng = float(wp_data.get('gps_lng', 0.0))
#                 waypoint.lat = float(wp_data.get('gps_lat', 0.0))

#                 # SLAM坐标系下的位姿
#                 waypoint.x = float(wp_data.get('slam_x', 0.0))
#                 waypoint.y = float(wp_data.get('slam_y', 0.0))
#                 waypoint.yaw = float(wp_data.get('slam_yaw', 0.0))

#                 # 航点类型
#                 waypoint.type = int(wp_data.get('type', 0))

#                 # 地图切换信息（仅当type为4时有效）
#                 if waypoint.type == 4:
#                     waypoint.next_map_name = str(wp_data.get('next_map_name', ''))
#                     waypoint.next_x = float(wp_data.get('next_slam_x', 0.0))
#                     waypoint.next_y = float(wp_data.get('next_slam_y', 0.0))
#                     waypoint.next_yaw = float(wp_data.get('next_slam_yaw', 0.0))
#                 else:
#                     waypoint.next_map_name = ''
#                     waypoint.next_x = 0.0
#                     waypoint.next_y = 0.0
#                     waypoint.next_yaw = 0.0

#                 # 容差
#                 waypoint.tolerance = float(wp_data.get('tolerance', 0.5))

#                 waypoint_list_msg.waypoints.append(waypoint)
                
#                 self.get_logger().info(
#                     f'  ✅ [{idx+1}/{waypoint_list_msg.total_waypoints}] '
#                     f'{waypoint.name} | '
#                     f'地图:{waypoint.map_name} | '
#                     f'GPS:({waypoint.lat:.6f},{waypoint.lng:.6f}) | '
#                     f'SLAM:({waypoint.x:.1f},{waypoint.y:.1f},{waypoint.yaw:.2f}) | '
#                     f'类型:{waypoint.type}'
#                 )

#             except (ValueError, TypeError) as e:
#                 self.get_logger().error(f'❌ 航点 {idx+1} 数据类型错误: {e}')
#                 self.get_logger().error(f'   数据: {wp_data}')
#                 continue
#             except Exception as e:
#                 self.get_logger().error(f'❌ 航点 {idx+1} 解析失败: {e}')
#                 self.get_logger().error(f'   数据: {wp_data}')
#                 continue

#         # 设置起始地图名称
#         if waypoint_list_msg.waypoints:
#             waypoint_list_msg.start_map_name = waypoint_list_msg.waypoints[0].map_name

#         self.get_logger().info(
#             f'✅ 解析完成: '
#             f'task_id={waypoint_list_msg.task_id}, '
#             f'waypoints={len(waypoint_list_msg.waypoints)}/{waypoint_list_msg.total_waypoints}, '
#             f'start_map={waypoint_list_msg.start_map_name}'
#         )

#         return waypoint_list_msg

#     def publish_to_ros(self, waypoint_list):
#         """将航点列表发布到ROS2主题"""
#         if waypoint_list.total_waypoints == 0:
#             self.get_logger().warn('⚠️  航点列表为空，不发布')
#             return
            
#         self.waypoint_list_pub.publish(waypoint_list)
#         self.get_logger().info(
#             f'📤 已发布到 /waypoint_list: '
#             f'{len(waypoint_list.waypoints)} 个航点'
#         )

#     def destroy_node(self):
#         """节点销毁时的清理"""
#         self.get_logger().info('正在关闭MQTT连接...')
#         self.mqtt_client.loop_stop()
#         self.mqtt_client.disconnect()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = MQTTTaskReceiver()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
"""
MQTT任务接收器节点
开发者A - 通信模块

订阅MQTT代理以接收TaskGraph任务并发布StartEndGraph到ROS2
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from std_msgs.msg import Header
from multi_map_navigation_msgs.msg import Waypoint, StartEndGraph
from multi_map_navigation_msgs.msg import Edge as EdgeMsg


class MQTTTaskReceiver(Node):
    def __init__(self):
        super().__init__('mqtt_task_receiver')

        # 声明参数
        self.declare_parameter('broker_url', 'localhost')
        self.declare_parameter('port', 1883)
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('client_id', 'robot_nav_001')
        self.declare_parameter('task_topic', 'prod/ctrl/vehicle/LS1234567890/task/start')
        self.declare_parameter('qos', 1)
        self.declare_parameter('reconnect_delay', 5.0)

        # 获取参数
        self.broker_url = self.get_parameter('broker_url').value
        self.port = self.get_parameter('port').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.client_id = self.get_parameter('client_id').value
        self.task_topic = self.get_parameter('task_topic').value
        self.qos = self.get_parameter('qos').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value

        # 连接状态
        self.is_connected = False

        # ROS2发布器 - 发布 StartEndGraph 消息
        self.graph_pub = self.create_publisher(
            StartEndGraph,
            '/start_end_graph',
            10
        )

        # MQTT客户端设置
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect

        # 如果提供了用户名和密码则设置
        if self.username and self.password:
            self.mqtt_client.username_pw_set(self.username, self.password)

        # 连接到代理
        self.connect_to_broker()

        # 在后台启动MQTT循环
        self.mqtt_client.loop_start()

        # 创建定时器检查连接状态
        self.reconnect_timer = self.create_timer(
            self.reconnect_delay,
            self.check_connection
        )

        self.get_logger().info('MQTT任务接收器已初始化')

    def connect_to_broker(self):
        """连接到MQTT代理"""
        try:
            self.get_logger().info(f'正在连接到MQTT代理 {self.broker_url}:{self.port}')
            self.mqtt_client.connect(self.broker_url, self.port, keepalive=60)
        except Exception as e:
            self.get_logger().error(f'连接MQTT代理失败: {e}')
            self.get_logger().info(f'将在 {self.reconnect_delay} 秒后重试...')

    def check_connection(self):
        """定期检查连接状态并尝试重连"""
        if not self.is_connected:
            self.get_logger().warn('未连接到MQTT代理，尝试重新连接...')
            try:
                self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().debug(f'重连失败: {e}')

    def on_connect(self, client, userdata, flags, rc):
        """连接到MQTT代理时的回调"""
        if rc == 0:
            self.is_connected = True
            self.get_logger().info('✅ 成功连接到MQTT代理')
            # 订阅任务主题
            client.subscribe(self.task_topic, qos=self.qos)
            self.get_logger().info(f'📡 已订阅主题: {self.task_topic}')
        else:
            self.is_connected = False
            error_messages = {
                1: '连接被拒绝 - 协议版本不正确',
                2: '连接被拒绝 - 客户端ID无效',
                3: '连接被拒绝 - 服务器不可用',
                4: '连接被拒绝 - 用户名或密码错误',
                5: '连接被拒绝 - 未授权'
            }
            error_msg = error_messages.get(rc, f'未知错误，返回码: {rc}')
            self.get_logger().error(f'❌ 连接MQTT代理失败: {error_msg}')

    def on_disconnect(self, client, userdata, rc):
        """从MQTT代理断开连接时的回调"""
        self.is_connected = False
        if rc != 0:
            self.get_logger().warn(f'⚠️  意外断开与MQTT代理的连接，返回码: {rc}')
            self.get_logger().info('🔄 将自动尝试重新连接...')
        else:
            self.get_logger().info('已正常断开MQTT连接')

    def on_message(self, client, userdata, msg):
        """从MQTT代理接收消息时的回调"""
        try:
            self.get_logger().info(f'📨 收到消息，主题: {msg.topic}')

            # 解析 JSON 字符串为 TaskGraph
            task_graph_data = json.loads(msg.payload.decode('utf-8'))

            # 验证数据格式：TaskGraph 应该是一个包含 points 和 edges 的字典
            if not isinstance(task_graph_data, dict):
                self.get_logger().error(
                    f'❌ 数据格式错误: 期望字典(TaskGraph)，收到 {type(task_graph_data).__name__}'
                )
                return

            if 'points' not in task_graph_data:
                self.get_logger().error('❌ 数据格式错误: 缺少 "points" 字段')
                return

            if 'edges' not in task_graph_data:
                self.get_logger().error('❌ 数据格式错误: 缺少 "edges" 字段')
                return

            points_data = task_graph_data['points']
            edges_data = task_graph_data['edges']

            if not isinstance(points_data, list) or len(points_data) < 2:
                self.get_logger().error(
                    f'❌ 数据格式错误: points 应为至少包含2个元素的列表，'
                    f'收到 {type(points_data).__name__}(长度:{len(points_data) if isinstance(points_data, list) else "N/A"})'
                )
                return

            if not isinstance(edges_data, list):
                self.get_logger().error(
                    f'❌ 数据格式错误: edges 应为列表，收到 {type(edges_data).__name__}'
                )
                return

            self.get_logger().info(
                f'📋 收到 TaskGraph: {len(points_data)} 个节点, {len(edges_data)} 条边'
            )

            # 解析并构建 StartEndGraph 消息
            start_end_graph_msg = self.parse_task_graph(points_data, edges_data)

            if start_end_graph_msg is not None:
                # 发布到ROS2
                self.publish_to_ros(start_end_graph_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ JSON解析失败: {e}')
            self.get_logger().error(
                f'原始消息: {msg.payload.decode("utf-8", errors="replace")[:200]}...'
            )
        except Exception as e:
            self.get_logger().error(f'❌ 处理消息时出错: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def parse_single_waypoint(self, wp_data, label=''):
        """
        将单个航点数据字典解析为 Waypoint 消息

        参数:
            wp_data: 航点数据字典
            label: 日志标签（如 "起点", "终点", "节点3"）

        返回:
            Waypoint 消息对象，解析失败返回 None
        """
        try:
            waypoint = Waypoint()

            # 基本信息
            waypoint.name = str(wp_data.get('name', ''))
            waypoint.id = int(wp_data.get('id', 0))

            # 地图信息
            waypoint.map_name = str(wp_data.get('map_name', ''))

            # GPS坐标
            waypoint.lng = float(wp_data.get('gps_lng', 0.0))
            waypoint.lat = float(wp_data.get('gps_lat', 0.0))

            # SLAM坐标系下的位姿
            waypoint.x = float(wp_data.get('slam_x', 0.0))
            waypoint.y = float(wp_data.get('slam_y', 0.0))
            waypoint.yaw = float(wp_data.get('slam_yaw', 0.0))

            # 航点类型
            waypoint.type = int(wp_data.get('type', 0))

            # 地图切换信息（仅当type为4时有效）
            if waypoint.type == 4:
                waypoint.next_map_name = str(wp_data.get('next_map_name', ''))
                waypoint.next_x = float(wp_data.get('next_slam_x', 0.0))
                waypoint.next_y = float(wp_data.get('next_slam_y', 0.0))
                waypoint.next_yaw = float(wp_data.get('next_slam_yaw', 0.0))
            else:
                waypoint.next_map_name = ''
                waypoint.next_x = 0.0
                waypoint.next_y = 0.0
                waypoint.next_yaw = 0.0

            # 容差
            waypoint.tolerance = float(wp_data.get('tolerance', 0.5))

            self.get_logger().info(
                f'  ✅ {label} '
                f'{waypoint.name}(id:{waypoint.id}) | '
                f'地图:{waypoint.map_name} | '
                f'GPS:({waypoint.lat:.6f},{waypoint.lng:.6f}) | '
                f'SLAM:({waypoint.x:.1f},{waypoint.y:.1f},{waypoint.yaw:.2f}) | '
                f'类型:{waypoint.type}'
            )

            return waypoint

        except (ValueError, TypeError) as e:
            self.get_logger().error(f'❌ {label} 数据类型错误: {e}')
            self.get_logger().error(f'   数据: {wp_data}')
            return None
        except Exception as e:
            self.get_logger().error(f'❌ {label} 解析失败: {e}')
            self.get_logger().error(f'   数据: {wp_data}')
            return None

    def parse_single_edge(self, edge_data, label=''):
        """
        将单条边数据字典解析为 Edge 消息

        参数:
            edge_data: 边数据字典
            label: 日志标签

        返回:
            EdgeMsg 消息对象，解析失败返回 None
        """
        try:
            edge_msg = EdgeMsg()

            edge_msg.start_node_id = int(edge_data.get('from_id', 0))
            edge_msg.end_node_id = int(edge_data.get('to_id', 0))
            edge_msg.weight = float(edge_data.get('weight', 1.0))

            # 检查边是否可用
            is_active = edge_data.get('is_active', True)
            if not is_active:
                self.get_logger().warn(
                    f'  ⚠️  {label} 边 {edge_data.get("id", "?")} 不可用(is_active=false)，已跳过'
                )
                return None

            self.get_logger().info(
                f'  ✅ {label} '
                f'id:{edge_data.get("id", "?")} | '
                f'{edge_msg.start_node_id} -> {edge_msg.end_node_id} | '
                f'权重:{edge_msg.weight:.2f}'
            )

            return edge_msg

        except (ValueError, TypeError) as e:
            self.get_logger().error(f'❌ {label} 数据类型错误: {e}')
            self.get_logger().error(f'   数据: {edge_data}')
            return None
        except Exception as e:
            self.get_logger().error(f'❌ {label} 解析失败: {e}')
            self.get_logger().error(f'   数据: {edge_data}')
            return None

    def parse_task_graph(self, points_data, edges_data):
        """
        将 TaskGraph 数据解析为 StartEndGraph 消息

        参数:
            points_data: 航点数据列表（第一个为起点，最后一个为终点）
            edges_data: 边数据列表

        返回:
            StartEndGraph 消息对象，解析失败返回 None
        """
        msg = StartEndGraph()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        total_points = len(points_data)
        total_edges = len(edges_data)

        self.get_logger().info(f'📊 开始解析 TaskGraph: {total_points} 个节点, {total_edges} 条边')

        # ===== 解析起点（第一个点）=====
        self.get_logger().info('--- 解析起点 ---')
        start_wp = self.parse_single_waypoint(points_data[0], label='[起点]')
        if start_wp is None:
            self.get_logger().error('❌ 起点解析失败，放弃此任务')
            return None
        msg.start = start_wp

        # ===== 解析终点（最后一个点）=====
        self.get_logger().info('--- 解析终点 ---')
        end_wp = self.parse_single_waypoint(points_data[-1], label='[终点]')
        if end_wp is None:
            self.get_logger().error('❌ 终点解析失败，放弃此任务')
            return None
        msg.end = end_wp

        # ===== 解析所有节点 =====
        self.get_logger().info('--- 解析所有节点 ---')
        for idx, wp_data in enumerate(points_data):
            waypoint = self.parse_single_waypoint(
                wp_data,
                label=f'[节点 {idx + 1}/{total_points}]'
            )
            if waypoint is not None:
                msg.nodes.append(waypoint)

        # ===== 解析所有边（仅保留 is_active=true 的边）=====
        self.get_logger().info('--- 解析所有边 ---')
        for idx, edge_data in enumerate(edges_data):
            edge = self.parse_single_edge(
                edge_data,
                label=f'[边 {idx + 1}/{total_edges}]'
            )
            if edge is not None:
                msg.edges.append(edge)

        self.get_logger().info(
            f'✅ TaskGraph 解析完成: '
            f'起点={msg.start.name}(id:{msg.start.id}), '
            f'终点={msg.end.name}(id:{msg.end.id}), '
            f'节点数={len(msg.nodes)}/{total_points}, '
            f'边数={len(msg.edges)}/{total_edges}'
        )

        return msg

    def publish_to_ros(self, start_end_graph_msg):
        """将 StartEndGraph 消息发布到ROS2主题"""
        if len(start_end_graph_msg.nodes) == 0:
            self.get_logger().warn('⚠️  节点列表为空，不发布')
            return

        self.graph_pub.publish(start_end_graph_msg)
        self.get_logger().info(
            f'📤 已发布到 /start_end_graph: '
            f'起点={start_end_graph_msg.start.name}, '
            f'终点={start_end_graph_msg.end.name}, '
            f'{len(start_end_graph_msg.nodes)} 个节点, '
            f'{len(start_end_graph_msg.edges)} 条边'
        )

    def destroy_node(self):
        """节点销毁时的清理"""
        self.get_logger().info('正在关闭MQTT连接...')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTTaskReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()