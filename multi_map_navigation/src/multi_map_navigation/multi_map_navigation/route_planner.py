import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from multi_map_navigation_msgs.msg import Waypoint, WaypointList, StartEndGraph
from multi_map_navigation_msgs.srv import PubNewPath
from std_msgs.msg import Header
import networkx as nx
import matplotlib.pyplot as plt

class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.G = None
        self.path_nodes = {} # 所有路径
        self.visited_paths = [] # 已经走过的路径

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
            
            nodes.append((node.name, node_attrs))
        
        for edge in msg.edges:
            edge_attrs= {"weight": edge.weight}
            edges.append((id_to_name[edge.start_node_id], id_to_name[edge.end_node_id], edge_attrs)) 

        # 添加节点
        self.G.add_nodes_from(nodes)

        # 添加边
        self.G.add_edges_from(edges)
        
        # 2.使用 shortest_simple_paths规划路径，它会自动按 weight 从小到大排序
        paths = nx.shortest_simple_paths(self.G, source=msg.start.name, target=msg.end.name, weight="weight")

        # 遍历所有的路径
        for idx, path in enumerate(paths):
            length = nx.path_weight(self.G, path, weight="weight")
            print(f"长度: {length:.3f} | 路径: {path}")

            # 为当前路径创建一个键，并初始化为空列表
            path_key = f"path{idx + 1}" 
            self.path_nodes[path_key] = []

            # 填充节点信息
            for node in path:
                print(f"  node: {node}, loc:{self.G.nodes[node]['lng']}, lat:{self.G.nodes[node]['lat']}")
                # 将节点加入到对应的路径列表中
                self.path_nodes[path_key].append(node)
        print(self.path_nodes)

        # 3. 发布第1条路径话题
        self.pub_waypoint_list("path1", 0, None)

    def pub_waypoint_list(self, path: str, switch_count, latest_switch_node):
        
        # 获取原始路径的所有节点名称
        full_path_nodes = self.path_nodes[path]

        # 根据 switch_count 处理路径截断
        if switch_count > 0 and latest_switch_node in full_path_nodes:
            # 找到最新切换点在当前路径中的索引位置
            start_index = full_path_nodes.index(latest_switch_node)
            # 截取从该点开始到结束的路径
            active_nodes = full_path_nodes[start_index:]
            # self.get_logger().info(f"路径 {path} 已截断，从切换点 {latest_switch_node} 开始执行剩余部分")
            print(f"路径 {path} 已截断，从切换点 {latest_switch_node} 开始执行剩余部分")
        else:
            # 如果没过切换点，或者切换点不在新路径中，则从头开始
            active_nodes = full_path_nodes

        # 填充waypoint_list路径消息
        waypoint_list_msg = WaypointList()
        waypoint_list_msg.header = Header()
        waypoint_list_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_list_msg.header.frame_id = 'map'

        # 生成任务ID
        waypoint_list_msg.task_id = f'task_{self.get_clock().now().to_msg().sec}'
        waypoint_list_msg.path = path

        for node_name in active_nodes:
            
            # 填充waypoint航点消息
            waypoint = self.fill_waypoint_msg_from_name(node_name)
            
            # 将填充好的航点加入列表
            waypoint_list_msg.waypoints.append(waypoint)
        
        waypoint_list_msg.total_waypoints = len(self.path_nodes[path])
        waypoint_list_msg.start_map_name = waypoint_list_msg.waypoints[0].map_name
        waypoint_list_msg.total_path = len(self.path_nodes)

        self.waypoint_list_pub.publish(waypoint_list_msg)
        # self.get_logger().info(f"共{waypoint_list_msg.total_path}条路径，已发布路径{path}，包含 {waypoint_list_msg.total_waypoints} 个航点")
        print(f"共{waypoint_list_msg.total_path}条路径，已发布路径{path}，包含 {waypoint_list_msg.total_waypoints} 个航点")
        
    def pub_new_path_callback(self, request, response):
        self.visited_paths.append(request.path_name)
        points = request.points

        # 计算剩余的路径
        remaining_keys = set(self.path_nodes.keys()) - set(self.visited_paths)
        
        # 如果所有路径都尝试过
        if len(list(remaining_keys)) == 0:
            response.success = False
            response.message = f'所有路径均尝试失败'
        
        # 如果还有其他备用路径
        else:
        
            # 提取已走过路径中所有的地图切换点
            switched_nodes = [p for p in points if p in self.G and self.G.nodes[p]['type'] == 4]

            # 获取经过的切换点数量
            switch_count = len(switched_nodes)

            # 如果需要获取最后一个（即最新的）切换点名称
            latest_switch_node = switched_nodes[-1] if switched_nodes else None

            # 如果到达过地图切换点，则以地图切换点为起点，使用下一条路径
            if switch_count > 0:
                self.get_logger().info(f"当前已过第 {switch_count} 个地图切换点，最新切换点为: {latest_switch_node}")
                
                # 逻辑：以最新的地图切换点为起点，重新计算到终点的路径
                self.pub_waypoint_list(list(remaining_keys)[0], switch_count, latest_switch_node)
            
            # 如果还未到达过地图切换点，则先回到起点，使用下一条路径
            else:
                self.pub_waypoint_list(list(remaining_keys)[0], switch_count, latest_switch_node)
            
            response.success = True
            response.message = f'已发布{list(remaining_keys)[0]}路径'

        print(f"服务调用成功，返回:{response}")
        return response

    def fill_waypoint_msg_from_name(self, node_name):
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