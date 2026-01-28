# 机器人综合状态消息定义
# 用于通过MQTT上报机器人的完整状态信息

std_msgs/Header header             # 时间戳和坐标系信息

# 机器人标识
string vin                    # 机器人唯一标识符

# 任务状态
int32 task_status                 # 任务状态: "idle" 或 "running"

# GPS位置信息
float64 gps_lng                    # GPS经度
float64 gps_lat                    # GPS纬度
float64 gps_alt                    # GPS高度（可选）

# 车辆状态
float32 cur_speed                  # 当前速度 (km/h)

# 电池状态
float32 battery_capacity           # 电池容量百分比 (0-100)
