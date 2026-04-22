import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from multi_map_navigation_msgs.msg import Waypoint, WaypointList, StartEndGraph
from multi_map_navigation_msgs.srv import PubNewPath
from std_msgs.msg import Header, String
import networkx as nx
import matplotlib.pyplot as plt
import math, time
import threading
from shapely.geometry import Point, MultiPoint
from sensor_msgs.msg import NavSatFix
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.G = None
        self.path_nodes = {} # 所有路径
        self.points = None # 已探索过的航点
        self.latest_gps = None # 最新gps数据 
        self.gps_lock = threading.Lock()# 互斥锁，防止多线程同时访问gps数据

        self.is_running = False # 机器人状态
        self.state_lock = threading.Lock()

        # 创建3个回调组，同一组内的回调不能同时执行，避免服务调用与订阅回调互相阻塞
        self.graph_cb_group = MutuallyExclusiveCallbackGroup()
        self.gps_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv_cb_group = MutuallyExclusiveCallbackGroup()

        # 创建订阅者: 订阅mqtt的起始点与图结构的
        self.start_end_graph_sub = self.create_subscription(
            StartEndGraph,
            '/start_end_graph',
            self.start_end_graph_callback,
            10,
            callback_group=self.graph_cb_group
        )

        # 创建订阅者，订阅GPS数据
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            '/sensing/gnss/pose_with_covariance', 
            self.gps_callback, 
            10,
            callback_group=self.gps_cb_group
        )

        # 创建订阅者，订阅机器人状态
        self.robot_state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            10
        )
        # 创建发布者: 发布计算后的路径点
        self.waypoint_list_pub = self.create_publisher(
            WaypointList,
            '/waypoint_list',
            # '/test/waypoint_list',
            10
        )

        # 创建服务端: 发布一条新路径
        self.pub_new_path_srv = self.create_service(
            PubNewPath,
            '/route_planner/pub_new_path',
            self.pub_new_path_callback,
            callback_group=self.srv_cb_group
        )
        self.get_logger().info('route_planner节点已初始化')
    
    def robot_state_callback(self, msg: String):
        """机器人任务状态回调 - 接收String类型，转换为int32存储"""
        # msg.data 是字符串: "idle" 或 "running"
        
        if msg.data == "idle":
            with self.state_lock:
                self.is_running = False
        else:
            with self.state_lock:
                self.is_running = True
            
    def gps_callback(self, msg: NavSatFix):
        with self.gps_lock:
            self.latest_gps = msg

    def start_end_graph_callback(self, msg: StartEndGraph):
        with self.state_lock:
            is_running = self.is_running
        
        if is_running == True: 
            self.get_logger().warn(f"机器人正在运行中, 无法规划路径")
            return

        # 1.创建最终图对象，一个空的无向图
        self.G = nx.Graph()
        # 节点id映射到节点名字
        id_to_name = {}
        nodes = []
        node_attrs = {}
        edges = []
        edge_attrs = {}
        
        self.start_node_name = msg.start.name
        self.end_node_name = msg.end.name
        # 把StartEndGraph消息中的节点信息添加到图中
        for node in msg.nodes:
            id_to_name[node.id] = node.name
            node_attrs= {
                        "name": node.name,
                        "map_name": node.map_name,
                        "id": node.id,
                        "lng": node.lng,
                        "lat": node.lat,
                        "x": node.x,
                        "y": node.y,
                        "yaw": node.yaw,
                        "type": node.type,
                        "next_map_name": node.next_map_name,
                        "next_x": node.next_x,
                        "next_y": node.next_y,
                        "next_yaw": node.next_yaw,
                        "tolerance": node.tolerance,
                    }
            # 如果 id 一样，则不添加 switch12_12, switch12_21
            nodes.append((node.name, node_attrs))
        
        for edge in msg.edges:
            edge_attrs= {"weight": edge.weight}
            edges.append((id_to_name[edge.start_node_id], id_to_name[edge.end_node_id], edge_attrs)) 

        # 添加节点
        self.G.add_nodes_from(nodes)

        # 添加边
        self.G.add_edges_from(edges)
        
        # 2.计算最短路径
        self.compute_path(self.start_node_name, self.end_node_name, "weight")
        print(f"[DEBG] 所有路径: {self.path_nodes}")

        # # 3.根据GPS判断机器人初始地图位置 
        # # TODO 判断经纬度正负
        # current_map_name = None
        # all_maps = set(attrs.get('map_name') for node, attrs in self.G.nodes(data=True) if attrs.get('map_name'))
        # max_attempts = 5
        # for attempt in range(max_attempts):
        #     # 每次尝试都重新获取一下最新的 GPS 数据
        #     with self.gps_lock:
        #         current_gps = self.latest_gps
            
        #     # 如果还没收到GPS信号，直接等下一轮
        #     if self.latest_gps is None:
        #         rclpy.spin_once(self, timeout_sec=1.0)
        #         self.get_logger().warn(f"尚未收到 GPS 数据，等待重试 ({attempt + 1}/{max_attempts})...")
        #         continue

        #     lon = current_gps.longitude
        #     lat = current_gps.latitude

        #     print(f"[DEBG] 机器人当前经纬度({lon},{lat})")
        #     # 遍历检查是否在某个地图内
        #     for map_name in all_maps:
        #         if self.is_robot_in_map_by_polygon(map_name, lon, lat):
        #             current_map_name = map_name
        #             self.get_logger().info(f"成功定位！机器人当前位于地图多边形内: {current_map_name}")
        #             break  # 跳出内层循环 (all_maps 循环)
            
        #     # 如果已经找到了，就直接跳出外层的重试循环
        #     if current_map_name is not None:
        #         break
            
        #     # 如果没找到，打印提示并稍微等一下再试（最后一次不用等）
        #     if attempt < max_attempts - 1:
        #         self.get_logger().warn(f"定位不在任何已知地图内，坐标({lon:.5f}, {lat:.5f})，稍后重试 ({attempt + 1}/{max_attempts})...")
        #         time.sleep(1.0) # 等待 1 秒，让 GPS 有时间刷新一下漂移

        # # 5次都尝试完了，还是没找到默认降级为起点地图。 TODO 上报
        # if current_map_name is None:
        #     current_map_name = self.G.nodes[self.start_node_name]['map_name']
        #     self.get_logger().warn(f"5次尝试均失败, 默认降级为起点地图: {current_map_name}。")
        #     return

        # 3.以第一个起始点所属地图为机器人当前所属地图
        current_map_name = self.G.nodes[self.start_node_name]["map_name"]

        # 4.更新路径中地图切换点属性
        self.update_graph_switch_node_from_list(self.path_nodes["path1"], current_map_name)

        # 5.发布第1条路径话题
        # self.pub_waypoint_list(path="path1", switch_count=0, latest_switch_node=None)
        self.pub_waypoint_list(self.path_nodes["path1"], path_name="path1")

    def pub_waypoint_list(self, node_list: list[str], path_name: str):
        '''
        function: 发布路点序列话题
        param: @ node_list: 路点列表
               @ path_name: 路径名称
        '''
        # 填充
        waypoint_list_msg = self.fill_way_point_list_msg(node_list, path_name)

        # 发布
        self.waypoint_list_pub.publish(waypoint_list_msg)

        # DEBG打印
        M_name_list = [p.name for p in waypoint_list_msg.waypoints if p.type == 4]
        for M_name in M_name_list:
            print(f"[DEBG] 地图切换点属性:{self.G.nodes[M_name]}")
        print(f"[DEBG] 发布一条路径:{node_list}")
        print(f"[DEBG] 共{waypoint_list_msg.total_path}条路径，已发布路径{path_name}，包含 {waypoint_list_msg.total_waypoints} 个航点")
        print("[DEBG] ---------***************-----------")
            
    def pub_new_path_callback(self, request, response):
        '''
        请求接收回调
        '''        
        print(f"[DEBG] 服务调用callback")
        # 保存上次导航已走过的点
        points = request.points
        if len(points) == 0:
            response.message = f'points路线为空'
            response.success = False
            print(f"[DEBG] 服务调用失败，返回:{response}")
            return response

        # 获取当前路径，path1 作为实际执行路径
        current_full_path_nodes_list = self.path_nodes["path1"]

        # 更新G节点地图，删除边，并重新规划路径 TODO 是否需要上报
        # 删除已走过的点与下一个点之间的边
        node1 = points[-1]
        node2_name =  current_full_path_nodes_list[current_full_path_nodes_list.index(node1.name) + 1]
        # self.G[node1.name][node2_name]['weight'] = math.inf
        self.G.remove_edge(node1.name, node2_name)
        
        # 以当前路径为起点，重新规划路径
        ret = self.compute_path(node1.name, self.end_node_name, "weight")
        if(ret == False):
            response.success = False
            response.message = f'所有路径均尝试失败'
            self.path_nodes = {}
            return response

        # 如果路径点内有地图切换点，需要更新节点对应的属性
        current_map_name  =  self.G.nodes[node2_name]['map_name']
        self.update_graph_switch_node_from_list(self.path_nodes["path1"], current_map_name)
        
        self.pub_waypoint_list(self.path_nodes["path1"], "path1")
        response.message = f'路径已下发'
        response.success = True
        print(f"[DEBG] 服务调用成功，返回:{response}")
        return response

    def fill_waypoint_msg_from_name(self, node_name: str):
        '''
        function: 将节点Node填充未Waypoint消息
        param:    @node_name: 路点名
        return:   Waypoint路点
        '''
        waypoint = Waypoint()

        # 从图中获取节点信息
        node_data = self.G.nodes[node_name]

        # 基本信息
        waypoint.name = node_name
        waypoint.id = node_data['id']
        waypoint.map_name = node_data['map_name']

        # GPS坐标
        waypoint.lng = node_data['lng']
        waypoint.lat = node_data['lat']

        # SLAM坐标系下的位姿
        waypoint.x = node_data['x']
        waypoint.y = node_data['y']
        waypoint.yaw = node_data['yaw']

        # 航点类型
        waypoint.type = node_data['type']
        
        # 容差
        waypoint.tolerance = node_data['tolerance']

        # 地图切换信息（仅当type为4时有效）
        if waypoint.type == 4:
            waypoint.next_map_name = node_data['next_map_name']
            waypoint.next_x = node_data['next_x']
            waypoint.next_y = node_data['next_y']
            waypoint.next_yaw = node_data['next_yaw']
        else:
            waypoint.next_map_name = ''
            waypoint.next_x = 0.0
            waypoint.next_y = 0.0
            waypoint.next_yaw = 0.0
        
        return waypoint
    
    def fill_way_point_list_msg(self, node_list: list[str], path_name: str):
        '''
        function: 根据路点列表与路径名填充waypoint_list
        param:    @node_list: 路点列表
                  @path_name: 路点名
        '''

        # 填充waypoint_list路径消息
        waypoint_list_msg = WaypointList()
        waypoint_list_msg.header = Header()
        waypoint_list_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_list_msg.header.frame_id = 'map'

        # 生成任务ID
        waypoint_list_msg.task_id = f'task_{self.get_clock().now().to_msg().sec}'
        waypoint_list_msg.path = path_name # 路线名称

        for node_name in node_list:
            
            # 填充waypoint航点消息
            waypoint = self.fill_waypoint_msg_from_name(node_name)
            
            # 将填充好的航点加入列表
            waypoint_list_msg.waypoints.append(waypoint)
        
        # 路点数量
        waypoint_list_msg.total_waypoints = len(node_list)
        
        # 起始地图名称
        waypoint_list_msg.start_map_name = waypoint_list_msg.waypoints[0].map_name 
        
        # 路线数量
        waypoint_list_msg.total_path = len(self.path_nodes) 
        return waypoint_list_msg

    def compute_path(self, source: str, target: str, weigth: str) -> bool :
        '''
        function: 计算最短路径，并保存为对象属性。
        param:    @source: 起点
                  @target: 终点
                  @weight: 权重字符串
        return: 返回失败则表示无路可走
        '''
        if nx.has_path(self.G,source=source, target=target):
        # 使用 shortest_simple_paths规划路径，它会自动按 weight 从小到大排序
        # 生成多条候选路径
            paths = nx.shortest_simple_paths(self.G, source=source, target=target, weight=weigth)
        else:
            return False

        idx = 0
        # 遍历所有的路径
        for path in paths:
            length = nx.path_weight(self.G, path, weight="weight")
            # print(f"[DEBG] 长度: {length:.3f} | 路径: {path}")

            # 为当前路径创建一个键，并初始化为空列表
            path_key = f"path{idx + 1}" 
            self.path_nodes[path_key] = []

            # 填充节点信息
            for node in path:
                # print(f"[DEBG] node: {node}, loc:{self.G.nodes[node]['lng']}, lat:{self.G.nodes[node]['lat']}")
                # 将节点加入到对应的路径列表中
                self.path_nodes[path_key].append(node)
            idx += 1
        return True
    
    def update_graph_switch_node_from_list(self, active_nodes_list:list[str], current_map: str):
        '''
        function: 遍历节点列表，更新地图切换节点的属性
        param :   @ activate_node_list: 节点列表
                  @ current_map: 机器人当前所属地图
        return: 更新后的列表
        '''
        for node in active_nodes_list:
            # 找到地图切换点
            if node in self.G and self.G.nodes[node]['type'] == 4:

                # 如果这个地图切换点在列表第一个位置，根据机器人当前地图位置判断地图切换点的属性
                if active_nodes_list.index(node) == 0:
                    # 如果机器人当前所使用地图与地图切换点的地图不同，则表示需要交换属性
                    if self.G.nodes[node]["map_name"] != current_map :
                        self.G.nodes[node]["map_name"], self.G.nodes[node]["next_map_name"] = self.G.nodes[node]["next_map_name"], self.G.nodes[node]["map_name"]
                        self.G.nodes[node]["x"], self.G.nodes[node]["next_x"] = self.G.nodes[node]["next_x"], self.G.nodes[node]["x"]
                        self.G.nodes[node]["y"], self.G.nodes[node]["next_y"] = self.G.nodes[node]["next_y"], self.G.nodes[node]["y"]
                        self.G.nodes[node]["yaw"], self.G.nodes[node]["next_yaw"] = self.G.nodes[node]["next_yaw"], self.G.nodes[node]["yaw"]
                    else:
                        pass

                # 如果地图切换点不在列表第一个位置
                else :
                    node_past = active_nodes_list[active_nodes_list.index(node) - 1]
                    print(f"[DEBG] 上一个路点: {node_past}, 上一个路点的地图名: {self.G.nodes[node_past]['map_name']}, 地图切换点的地图名:{self.G.nodes[node]['map_name']}")
                    # 如果这个地图切换点与上一个节点的地图名不同，则表示需要交换属性
                    if self.G.nodes[node]["map_name"] != self.G.nodes[node_past]["map_name"]:
                        print(f"[DEBG] 切换属性")
                        self.G.nodes[node]["map_name"], self.G.nodes[node]["next_map_name"] = self.G.nodes[node]["next_map_name"], self.G.nodes[node]["map_name"]
                        self.G.nodes[node]["x"], self.G.nodes[node]["next_x"] = self.G.nodes[node]["next_x"], self.G.nodes[node]["x"]
                        self.G.nodes[node]["y"], self.G.nodes[node]["next_y"] = self.G.nodes[node]["next_y"], self.G.nodes[node]["y"]
                        self.G.nodes[node]["yaw"], self.G.nodes[node]["next_yaw"] = self.G.nodes[node]["next_yaw"], self.G.nodes[node]["yaw"]
                    else:
                        pass

    def is_robot_in_map_by_polygon(self, target_map_name: str, robot_lon: float, robot_lat: float) -> bool:
        """
        function: 判断经纬度是否在指定地图内
        param: @ target_map_name: 地图名称
               @ robot_lon: 经度
               @ robot_lat: 纬度
        """
        # 1. 收集目标地图所有节点的坐标
        map_points = []
        for node_name, attrs in self.G.nodes(data=True):
            if attrs.get('map_name') == target_map_name or attrs.get('next_map_name') == target_map_name:
                # 注意 shapely 通常按 (经度/X, 纬度/Y) 排列
                map_points.append((attrs.get('lng', 0.0), attrs.get('lat', 0.0)))
        
        # print(f"[DEBG] map_name: {target_map_name}, map_points:{map_points}")
        if len(map_points) < 3:
            # 点太少构不成面，退化为判断是否在点附近（省略实现）
            self.get_logger().warn(f"地图 {target_map_name} 节点少于3个, 无法构建多边形")
            return False

        # 2. 构建包含这些点的外包围多边形 (凸包)
        # MultiPoint 会把所有点变成一个几何集合，.convex_hull 会自动用“橡皮筋”把它们的最外围连起来
        map_polygon = MultiPoint(map_points).convex_hull
        # print(f"[DEBG] map_polygon:{map_polygon}, robot_lng, robot_lat:{robot_lon, robot_lat}")

        # 3. 扩大一点边界 (加个缓冲)，防止机器人在边界线上产生误判
        # 注意：经纬度下的 0.0001 度大概相当于 10 米左右
        map_polygon_with_buffer = map_polygon.buffer(0.0001) 
        
        # 4. 判断机器人当前点是否在多边形内
        robot_point = Point(robot_lon, robot_lat)
        return map_polygon_with_buffer.contains(robot_point)

def main(args=None):
    rclpy.init(args=args)
    node = RoutePlannerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()