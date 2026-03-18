import networkx as nx
import matplotlib.pyplot as plt
import math

# 创建一个空的无向图
G = nx.Graph()

nodes = [
    ("start", {"loc": 103.01, "lat": 30.01}),
    ("B1", {"loc": 103.02, "lat": 30.02}),
    ("C1", {"loc": 103.03, "lat": 30.03}),
    ("end", {"loc": 103.04, "lat": 30.04}),
    ("B2", {"loc": 103.05, "lat": 30.05}),
    ("C2", {"loc": 103.06, "lat": 30.06}),
]

edges = [
    ("start", "B1",{"weight": math.inf}),
    ("B1", "C1",{"weight": 0.3}),
    ("C1", "end",{"weight": 0.2}),
    ("start", "B2",{"weight": 0.3}),
    ("B2", "C2",{"weight": 0.1}),
    ("C2", "end",{"weight": 0.1}),
]

path_nodes={}
# {"path1":
#  [
#      node1,
#      node2,
#      ...
#  ],
#  "path2":
#  [
#     node1,
#     node3,
#     ...
#  ]
#  }

# 添加节点
G.add_nodes_from(nodes)

# 添加边
G.add_edges_from(edges)

# 使用 shortest_simple_paths规划路径，它会自动按 weight 从小到大排序
paths = nx.shortest_simple_paths(G, source="start", target="end", weight='weight')
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

def find_inf_path(path_node_dict: dict, weight: str) -> list[dict]:
    '''
    function: 找到长度为无穷的路径名
    param:    
    '''
    inf_path_node_dict = {}
    for path_name, node_name_list in path_node_dict.items():
        length = nx.path_weight(G, node_name_list, weight)
        if length == math.inf:
            inf_path_node_dict[path_name] = node_name_list
    return inf_path_node_dict

inf_path_node_dict = find_inf_path(path_nodes, "weight")
print(f"无穷路径: {inf_path_node_dict}")
# 绘制网络图
nx.draw(G, with_labels=True, node_color='skyblue', node_size=700, edge_color='k')
plt.show()