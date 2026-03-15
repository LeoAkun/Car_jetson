# 航点列表消息定义
# 包含用于多地图导航的航点序列

std_msgs/Header header             # 时间戳和坐标系信息

Waypoint[] waypoints               # 要导航的航点数组

# 任务元数据
string task_id                     # 此任务的唯一标识符
string path                        # 路径名称，如path1,path2,...
int32 total_waypoints              # 航点总数
string start_map_name              # 起始地图名称
int32 total_path                   # 路径总数
