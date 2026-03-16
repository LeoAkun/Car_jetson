import networkx as nx
import matplotlib.pyplot as plt
import math

# 创建一个空的无向图
G = nx.Graph()

nodes = [
    ("A", {"loc": 103.01, "lat": 30.01}),
    ("B", {"loc": 103.02, "lat": 30.02}),
    ("switch12_12", {"map": "map1", "next_map": "map2"}),
    ("switch12_21", {"map": "map2", "next_map": "map1"})

]

edges = [
    ("A", "B",{"weight": 0.2}),
    ("B", "switch12_12",{"weight": 0.3}),
]

path_nodes={}

# 添加节点
G.add_nodes_from(nodes)

# 添加边
G.add_edges_from(edges)

# 使用 shortest_simple_paths规划路径，它会自动按 weight 从小到大排序
paths = nx.shortest_simple_paths(G, source="A", target="switch12_12", weight='weight')
idx = 0
# 遍历所有的路径
for idx, path in enumerate(paths):
    length = nx.path_weight(G, path, weight="weight")
    # print(f"长度: {length:.3f} | 路径: {path}")

    # 为当前路径创建一个键，并初始化为空列表
    path_key = f"path{idx + 1}" 
    path_nodes[path_key] = []

    # 填充节点信息
    for node in path:
        # print(f"  node: {node}, loc:{self.G.nodes[node]['lng']}, lat:{self.G.nodes[node]['lat']}")
        # 将节点加入到对应的路径列表中
        path_nodes[path_key].append(node)
print(f"路径: {path_nodes}")

# 绘制网络图
nx.draw(G, with_labels=True, node_color='skyblue', node_size=700, edge_color='k')
# plt.show()

plt.savefig("network_graph.png", dpi=150, bbox_inches='tight')

# 交换属性
attr12 = G.nodes['switch12_12'].copy()
attr21 = G.nodes['switch12_21'].copy()
G.nodes['switch12_12'].update(attr21)
G.nodes['switch12_21'].update(attr12)

print(f"switch12_12属性: {G.nodes['switch12_12']}")