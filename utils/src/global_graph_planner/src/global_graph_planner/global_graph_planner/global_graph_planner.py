import rclpy
import rclpy.node
from global_route_msg.msg import Node
from global_route_msg.srv import GlobalPlanRoute

import osmnx as ox
import networkx as nx
import os
import shutil
import matplotlib.pyplot as plt

class GlobalGraphPlanner(rclpy.node.Node): 
    def __init__(self, name):
        super().__init__(node_name=name)
        
        self.service = self.create_service(GlobalPlanRoute, "CacuGlobalPlanService", self.cacu_planner_callback)
        self.get_logger().info('✅ 服务 [CacuGlobalPlanService] 已启动')

    def cacu_planner_callback(self, request, response):

        # 1️⃣ 删除缓存目录
        cache_path = "./cache"
        if os.path.exists(cache_path):
            shutil.rmtree(cache_path)

        # 2️⃣ 获取路网
        place = "Southwest Jiaotong University (Xipu Campus)"
        G = ox.graph_from_place(place, network_type='bike', simplify=False)

        # 3️⃣ 起点终点
        # "天佑斋17号楼"
        # "西南交大犀浦3号教学楼"
        start_point = ox.geocoder.geocode(request.start_place_name)
        end_point = ox.geocoder.geocode(request.dest_place_name)

        # 4️⃣ 找到最近节点
        orig = ox.distance.nearest_nodes(G, X=start_point[1], Y=start_point[0])
        dest = ox.distance.nearest_nodes(G, X=end_point[1], Y=end_point[0])

        # 5️⃣ 最短路径
        route = nx.shortest_path(G, orig, dest, weight='length') # WSG84坐标系。如果要在高德地图显示的话需要转为GCJ-02坐标系
        
        for i, point in enumerate(route):
            node = Node()
            node.id = point
            node.latitude = G.nodes[point]['y']   # 纬度
            node.longtitude = G.nodes[point]['x'] # 经度
            response.points.append(node)
        
        self.get_logger().info('✅ 服务 [CacuGlobalPlanService] 调用成功！')
        return response
        # print(self.route)
        
        # print("Route node IDs:", route)

        # 打印每个节点的经纬度
        # print("Route coordinates (lat, lon):")
        # for node in route:
        #     lat = G.nodes[node]['y']
        #     lon = G.nodes[node]['x']
        #     print(f"  ({lat:.6f}, {lon:.6f})")

        # # 6️⃣ 绘制路径 + 节点
        # # 先绘制路径
        # fig, ax = ox.plot_graph_route(
        #     G, route, route_color='red', route_linewidth=4, node_size=0,
        #     bgcolor='white', show=False, close=False
        # )

        # # 再叠加节点
        # route_nodes = [(G.nodes[n]['x'], G.nodes[n]['y']) for n in route]
        # xs, ys = zip(*route_nodes)

        # # 绘制起点终点、路径节点
        # ax.scatter(xs, ys, c='blue', s=30, zorder=5, label='Path nodes')
        # ax.scatter(xs[0], ys[0], c='green', s=80, marker='o', label='Start')
        # ax.scatter(xs[-1], ys[-1], c='red', s=80, marker='X', label='End')
        
        # ax.legend()
        # plt.show()

def main(args=None):
    rclpy.init(args=args)
    globalPlanner = GlobalGraphPlanner('globalGraphPlanner')
    rclpy.spin(globalPlanner)
    globalPlanner.destroy_node()
    rclpy.shutdown()
