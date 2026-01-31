# 航点消息定义
# 表示导航序列中的单个航点

# 航点基本信息
string name                        # 航点名称
int32 id                          # 航点唯一标识符

# 地图信息
string map_name                    # 此航点所属的地图名称

# GPS坐标
float64 lng                        # 经度
float64 lat                        # 纬度

# 当前地图坐标系下的位姿
float64 x                          # 当前地图坐标系下的X坐标
float64 y                          # 当前地图坐标系下的Y坐标
float64 yaw                        # 当前地图坐标系下的偏航角（弧度）

# 航点类型
int32 type                         # 类型: 1=正常导航点, 2=红绿灯, 3=充电桩, 4=地图切换点

# 地图切换信息（仅当type为1时有效）
string next_map_name               # 要切换到的下一个地图名称
float64 next_x                     # 当前点在下一张地图坐标系下的X坐标
float64 next_y                     # 当前点在下一张地图坐标系下的Y坐标
float64 next_yaw                   # 当前点在下一张地图坐标系下的偏航角（弧度）

# 附加参数
float32 tolerance                  # 目标容差（米）（可选，默认从配置读取）
