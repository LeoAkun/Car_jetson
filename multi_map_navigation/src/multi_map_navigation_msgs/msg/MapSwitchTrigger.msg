# 地图切换触发消息定义
# 触发地图切换操作

std_msgs/Header header             # 时间戳

# 地图信息
string current_map                 # 当前地图名称
string next_map                    # 要切换到的下一个地图名称

# 切换位姿信息
geometry_msgs/PoseStamped switch_pose  # 地图切换点的位姿

# 航点信息
int32 current_waypoint_id          # 当前航点（切换点）的ID
int32 next_waypoint_id             # 切换后的下一个航点ID
