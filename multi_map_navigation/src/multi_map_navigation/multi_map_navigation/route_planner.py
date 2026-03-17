import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from multi_map_navigation_msgs.msg import Waypoint, WaypointList, StartEndGraph
from multi_map_navigation_msgs.srv import PubNewPath
from std_msgs.msg import Header
import networkx as nx
import matplotlib.pyplot as plt
import math
class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.G = None
        self.path_nodes = {} # 所有路径
        self.points = None # 已探索过的航点

        # 创建订阅者: 订阅mqtt的起始点与图结构的
        self.start_end_graph_sub = self.create_subscription(
            StartEndGraph,
            '/start_end_graph',
            self.start_end_graph_callback,
            10
        )

        # 创建发布者: 发布计算后的路径点
        self.waypoint_list_pub = self.create_publisher(
            WaypointList,
            '/waypoint_list',
            10
        )
        # 创建服务端: 发布一条新路径
        self.pub_new_path_srv = self.create_service(
            PubNewPath,
            '/route_planner/pub_new_path',
            self.pub_new_path_callback
        )
    
    def start_end_graph_callback(self, msg: StartEndGraph):

        # 1.创建一个空的无向图
        self.G = nx.Graph()
        id_to_name = {}
        nodes = []
        node_attrs = {}
        edges = []
        edge_attrs = {}
        
        self.start_node_name = msg.start.name
        self.end_node_name = msg.end.name
        
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

        # 3. 发布第1条路径话题
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
        print(f"[DEBG] 发布一条路径:{node_list}")
        print(f"[DEBG] 共{waypoint_list_msg.total_path}条路径，已发布路径{path_name}，包含 {waypoint_list_msg.total_waypoints} 个航点")
        print("[DEBG] ---------***************-----------")
            
    def pub_new_path_callback(self, request, response):
        '''
        请求接收回调
        '''        

        # 保存上次导航已走过的点
        points = request.points

        # 获取当前路径
        current_full_path_nodes_list = self.path_nodes["path1"]

        # 更新G节点地图，删除边，并重新规划路径 TODO 是否需要上报
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
        self.update_graph_switch_node_from_list(self.path_nodes["path1"], self.G.nodes[node1.name]["map_name"])
        
        print(f"[DEBG] 地图切换点属性:{self.G.nodes['M']}")
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
        param :   @activate_node_list: 节点列表
        return: 更新后的列表
        '''
        for node in active_nodes_list:
            # 找到地图切换点
            if node in self.G and self.G.nodes[node]['type'] == 4:

                # 如果这个地图切换点再列表第一个位置，根据机器人当前地图位置判断地图切换点的属性
                if active_nodes_list.index(node) == 0:
                    # 如果机器人当前所使用地图与地图切换点的地图不同，则表示需要交换属性
                    if self.G.nodes[node]["map_name"] != current_map :
                        self.G.nodes["map_name"], self.G.nodes["next_map_name"] = self.G.nodes["next_map_name"], self.G.nodes["map_name"]
                        self.G.nodes["x"], self.G.nodes["next_x"] = self.G.nodes["next_x"], self.G.nodes["x"]
                        self.G.nodes["y"], self.G.nodes["next_y"] = self.G.nodes["next_y"], self.G.nodes["y"]
                        self.G.nodes["yaw"], self.G.nodes["next_yaw"] = self.G.nodes["next_yaw"], self.G.nodes["yaw"]
                    else:
                        pass

                # 如果地图切换点不在列表第一个位置
                else :

                    # 如果这个地图切换点与上一个节点的地图名不同，则表示需要交换属性
                    node_past = active_nodes_list[active_nodes_list.index(node) - 1]
                    if self.G.nodes[node]["map_name"] != self.G.nodes[node_past]["map_name"]:
                        self.G.nodes[node]["map_name"], self.G.nodes[node]["next_map_name"] = self.G.nodes["next_map_name"], self.G.nodes["map_name"]
                        self.G.nodes[node]["x"], self.G.nodes[node]["next_x"] = self.G.nodes["next_x"], self.G.nodes["x"]
                        self.G.nodes[node]["y"], self.G.nodes[node]["next_y"] = self.G.nodes["next_y"], self.G.nodes["y"]
                        self.G.nodes[node]["yaw"], self.G.nodes[node]["next_yaw"] = self.G.nodes["next_yaw"], self.G.nodes["yaw"]
                    else:
                        pass

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