# MQTT给planner下发起始点，图结构信息

std_msgs/Header header             # 时间戳和坐标系信息
Waypoint start  # 起始点
Waypoint end    # 目标点
# Graph g         # 图结构
Waypoint[] nodes  # 图的所有节点
Edge[] edges       # 图的所有边
