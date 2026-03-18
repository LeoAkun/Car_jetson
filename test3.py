from shapely.geometry import Point, MultiPoint
import networkx as nx
import matplotlib.pyplot as plt
import math

# 创建一个空的无向图
G = nx.Graph()

nodes = [
    ("A", {"lng": 100, "lat": 30, 'map_name': "map1"}),
    ("B", {"lng": 100, "lat": 35, 'map_name': "map1"}),
    ("C", {"lng": 105, "lat": 30, 'map_name': "map1"}),
    ("D", {"lng": 98, "lat": 32, 'map_name': "map1"}),
]


# 添加节点
G.add_nodes_from(nodes)

def is_robot_in_map_by_polygon(target_map_name: str, robot_lat: float, robot_lon: float) -> bool:
    """
    判断机器人是否在指定地图内（基于多边形边界）
    """
    # 1. 收集目标地图所有节点的坐标
    map_points = []
    for node_name, attrs in G.nodes(data=True):
        if attrs.get('map_name') == target_map_name:
            # 注意 shapely 通常按 (经度/X, 纬度/Y) 排列
            map_points.append((attrs.get('lng', 0.0), attrs.get('lat', 0.0)))
            
    if len(map_points) < 3:
        # 点太少构不成面，退化为判断是否在点附近（省略实现）
        print(f"地图 {target_map_name} 节点少于3个，无法构建多边形")
        return False

    # 2. 构建包含这些点的外包围多边形 (凸包)
    # MultiPoint 会把所有点变成一个几何集合，.convex_hull 会自动用“橡皮筋”把它们的最外围连起来
    map_polygon = MultiPoint(map_points).convex_hull
    
    # 3. 扩大一点边界 (加个缓冲)，防止机器人在边界线上产生误判
    # 注意：经纬度下的 0.0001 度大概相当于 10 米左右
    map_polygon_with_buffer = map_polygon.buffer(0.0001) 
    
    # 4. 判断机器人当前点是否在多边形内
    robot_point = Point(robot_lon, robot_lat)
    return map_polygon_with_buffer.contains(robot_point)

print(is_robot_in_map_by_polygon("map1", 32, 99))