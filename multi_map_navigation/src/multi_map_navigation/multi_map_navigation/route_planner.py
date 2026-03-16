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
        
        # TODO 当遇到两个地图切换点时，拓扑图上只保留一个，另一个是单独的节点没有任何边
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
        print(self.path_nodes)

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
        print(f"发布一条路径:{node_list}")
        print(f"共{waypoint_list_msg.total_path}条路径，已发布路径{path_name}，包含 {waypoint_list_msg.total_waypoints} 个航点")
        print("---------***************-----------")
            
    def pub_new_path_callback(self, request, response):
        '''
        请求接收回调
        '''        

        # 保存上次导航已走过的点
        points = request.points

        # 获取当前路径
        current_path_name = request.path_name
        current_full_path_nodes_list = self.path_nodes[current_path_name]

        # 更新G节点地图，将边的权重设置为无穷，并重新规划路径 TODO 是否需要上报
        node1 = points[-1]
        node2 =  current_full_path_nodes_list[current_full_path_nodes_list.index(node1) + 1]
        self.G[node1.name][node2.name]['weight'] = math.inf
        # self.G.remove_edge(node1.name, node2.name)
        inf_path_dict = self.find_inf_path(self.path_nodes, 'weight')
        
        # 计算剩余的路径: 去除掉长度为无穷的路径
        remaining_path_name_list = list(set(self.path_nodes.keys()) - set(inf_path_dict.keys()))

        # 如果所有路径都尝试过
        if len(remaining_path_name_list) == 0:
            response.success = False
            response.message = f'所有路径均尝试失败'
            self.points = None
            return response
        
        # 寻找备用路径，这个路径需要包含至少一个已经走过的点
        if self.points is None:    
            self.points = points[:-2]
        for node_name in [point.name for point in self.points][::-1]:
            for path_name in remaining_path_name_list
                if node_name in self.path_nodes[path_name]:
                    new_path_name = path_name
                    break
        
        new_full_path_nodes_list = self.path_nodes[new_path_name]
        
        # 提取已走过的点中，所有的地图切换点
        switched_nodes_name_list = [p.name for p in self.points if p.name in self.G and self.G.nodes[p.name]['type'] == 4]

        # 获取经过的切换点数量
        switch_count = len(switched_nodes_name_list)

        # 获取最后一个（即最新的）切换点名称
        latest_switch_node_name = switched_nodes_name_list[-1] if switched_nodes_name_list else None

        # 如果已经走过地图切换点， 逻辑相同? TODO
        if switch_count > 0 and latest_switch_node_name in current_full_path_nodes_list:
            
            # 断路点
            start_node_name = points[-1].name

            # 找与备用新路径的最近交点
            common_nodes_name_list = [p.name for p in self.points if p.name in self.path_nodes[new_path_name]]
            common_nodes_name = common_nodes_name_list[-1]

            # 计算返回从“断路点”到“交点”的路径航点
            node_name_list = [node.name for node in self.points]
            return_path_nodes_list = self.compute_node_list_from_start_end(start_node_name, common_nodes_name, node_name_list)

            # 计算新路径剩余航点
            idx_common_nodes_name_in_new = new_full_path_nodes_list.index(common_nodes_name)
            remain_path_nodes_list = new_full_path_nodes_list[idx_common_nodes_name_in_new + 1:]

            # 从该地图切换点开始
            active_nodes_list = return_path_nodes_list + remain_path_nodes_list

            # self.get_logger().info(f"路径 {path} 已截断，从切换点 {latest_switch_node} 开始执行剩余部分")
            response.message = f'已发布{new_path_name}路径，经过地图切换点，先回到切换点 {latest_switch_node_name}, 然后执行剩余部分'
            print(f'已发布{new_path_name}路径，经过地图切换点，先回到切换点 {latest_switch_node_name}, 然后执行剩余部分')

        # 如果没有走过地图切换点
        else:
            # 断路点 
            start_node_name = points[-1].name
            
            # 找与备用路径的交点
            common_nodes_name_list = [p.name for p in self.points if p.name in self.path_nodes[new_path_name]]
            common_nodes_name = common_nodes_name_list[-1]

            # 计算返回从“断路点”到“交点”的路径航点
            node_name_list = [node.name for node in points]
            return_path_nodes_list = self.compute_node_list_from_start_end(start_node_name, common_nodes_name, node_name_list)

            # 计算新路径剩余航点
            idx_common_nodes_name_in_new = new_full_path_nodes_list.index(common_nodes_name)
            remain_path_nodes_list = new_full_path_nodes_list[idx_common_nodes_name_in_new + 1:]

            # 如果没过切换点，或者切换点不在新路径中，则回到起点从头开始
            active_nodes_list = return_path_nodes_list + remain_path_nodes_list

            response.message = f'已发布{new_path_name}路径，未经过地图切换点，先回到公共点 {common_nodes_name} , 然后执行剩余部分'
            print(f'已发布{new_path_name}路径，未经过地图切换点，先回到公共点 {common_nodes_name}, 然后执行剩余部分')
        # 发布话题

        # 如果路径点内有地图切换点，需要判断地图切换的方向！ TODO
        for p in active_nodes_list:
            # 找到所有的地图切换点
            if p in self.G and self.G.nodes[p]['type'] == 4:
                # 前一个点存在, 则切换点的地图为上一个点的地图
                if active_nodes_list.index(p) > 0:

                    # 只交换属性，不交换节点名字，fill_waypoint赋值的属性的为交换后的。
                    attr12 = G.nodes['switch12_12'].copy()
                    attr21 = G.nodes['switch12_21'].copy()
                    G.nodes['switch12_12'].update(attr21)
                    G.nodes['switch12_21'].update(attr12)

                # 如果前一个点不存在，则切换点的地图为机器人当前使用的地图
                else :
                    p

        switched_nodes_name_list = [p.name for p in active_nodes_list if p.name in self.G and self.G.nodes[p.name]['type'] == 4]

        self.pub_waypoint_list(active_nodes_list, new_path_name)

        # 更新已走过的点
        self.points = self.points[:self.points.index(node_name) + 1] # 切片左闭右开

        response.success = True
        print(f"服务调用成功，返回:{response}")
        return response
    
    def compute_node_list_from_start_end(self, start_node_name: str, end_node_name: str, path_nodes_list: list[str]) -> list[str]: 
        """
        function: 在指定路径中截取起始点到终点之间的节点序列。支持正向截取和反向回溯。
        param:    @start_node_name: 起始路点名
                  @end_node_name: 终点路点名
                  @path_nodes_list: 待查询的路点列表
        return:   路点序列
        """
        # 1. 获取该路径名称对应的完整节点列表
        full_nodes_name_list = path_nodes_list
        
        if start_node_name not in full_nodes_name_list or end_node_name not in full_nodes_name_list:
            self.get_logger().warn(f"节点 {start_node_name} 或 {end_node_name} 不在路径中")
            return []

        # 2. 找到两个节点在列表中的索引位置
        idx_start = full_nodes_name_list.index(start_node_name)
        idx_end = full_nodes_name_list.index(end_node_name)

        # 3. 根据索引顺序截取
        if idx_start <= idx_end:
            # 正向截取：比如从 B(idx=1) 到 D(idx=3) -> [B, C, D]
            # Python 切片是左闭右开，所以要 +1
            return full_nodes_name_list[idx_start : idx_end + 1]
        else:
            # 反向回溯：比如从 D(idx=3) 到 B(idx=1) -> [D, C, B]
            # 先切出 [B, C, D]，然后使用 [::-1] 反转
            return full_nodes_name_list[idx_end : idx_start + 1][::-1]

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

    def compute_path(self, source: str, target: str, weigth: str):
        '''
        function: 计算最短路径，并保存为对象属性。
        param:    @source: 起点
                  @target: 终点
                  @weight: 权重字符串
        '''
        
        # 使用 shortest_simple_paths规划路径，它会自动按 weight 从小到大排序
        paths = nx.shortest_simple_paths(self.G, source=source, target=target, weight=weigth)
        idx = 0
        # 遍历所有的路径
        for path in paths:
            length = nx.path_weight(self.G, path, weight="weight")
            # print(f"长度: {length:.3f} | 路径: {path}")

            # 为当前路径创建一个键，并初始化为空列表
            path_key = f"path{idx + 1}" 
            self.path_nodes[path_key] = []

            # 填充节点信息
            for node in path:
                # print(f"  node: {node}, loc:{self.G.nodes[node]['lng']}, lat:{self.G.nodes[node]['lat']}")
                # 将节点加入到对应的路径列表中
                self.path_nodes[path_key].append(node)
            idx += 1
    
    def find_inf_path(self, path_nodes_dict: dict, weight: str) -> dict:
        '''
        function: 找到长度为无穷的路径名
        param:    
        '''
        inf_path_nodes_dict = {}
        for path_name, node_name_list in path_nodes_dict.items():
            length = nx.path_weight(self.G, node_name_list, weight)
            if length == math.inf:
                inf_path_nodes_dict[path_name] = node_name_list
        return inf_path_nodes_dict
    
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