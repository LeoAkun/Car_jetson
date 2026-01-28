# RobotStatus 消息使用说明

## 概述

`RobotStatus.msg` 是一个综合的机器人状态消息，用于通过MQTT上报机器人的完整状态信息。

## 消息定义

```msg
std_msgs/Header header             # 时间戳和坐标系信息

# 机器人标识
string robot_id                    # 机器人唯一标识符

# 任务状态
string task_status                 # 任务状态: "idle" 或 "running"

# GPS位置信息
float64 gps_lng                    # GPS经度
float64 gps_lat                    # GPS纬度
float64 gps_alt                    # GPS高度（可选）

# 车辆状态
float32 cur_speed                  # 当前速度 (km/h)

# 电池状态
float32 battery_capacity           # 电池容量百分比 (0-100)
```

## 数据来源

该消息整合了以下ROS2话题的数据：

| 字段 | 数据来源话题 | 消息类型 | 说明 |
|------|-------------|---------|------|
| `robot_id` | 配置参数 | - | 从mqtt_config.yaml读取 |
| `task_status` | `/robot_state` | std_msgs/String | idle或running |
| `gps_lng`, `gps_lat`, `gps_alt` | `/sensing/gnss/pose_with_covariance` | geometry_msgs/PoseWithCovarianceStamped | GPS经纬度和高度 |
| `cur_speed` | `/vehicle_status` | yunle_msgs/VehicleStatus | 当前车速(km/h) |
| `battery_capacity` | `/battery_status` | sensor_msgs/BatteryState | 电池容量百分比 |

## 使用方法

### 1. 编译消息

```bash
cd /home/akun/workspace/Car_jetson/multi_map_navigation
colcon build --packages-select multi_map_navigation_msgs
source install/setup.bash
```

### 2. 配置机器人ID

编辑 `config/mqtt_config.yaml`:

```yaml
mqtt:
  robot_id: "robot_001"  # 修改为你的机器人ID
```

### 3. 启动状态上报节点

```bash
ros2 launch multi_map_navigation multi_map_navigation.launch.py
```

或单独启动状态报告器：

```bash
ros2 run multi_map_navigation status_reporter --ros-args \
  --params-file config/mqtt_config.yaml
```

### 4. 订阅ROS2话题

```bash
# 查看RobotStatus消息
ros2 topic echo /robot_status
```

### 5. MQTT消息格式

发送到MQTT的JSON格式：

```json
{
  "robot_id": "robot_001",
  "task_status": "running",
  "gps_lng": 116.397128,
  "gps_lat": 39.916527,
  "gps_alt": 50.5,
  "cur_speed": 5.2,
  "battery_capacity": 85.3,
  "timestamp": 1706428800
}
```

## 注意事项

### GPS坐标转换

`/sensing/gnss/pose_with_covariance` 话题的 `position.x` 和 `position.y` 可能存储的是：
- **经纬度坐标**：直接使用
- **UTM坐标**：需要转换为经纬度

如果是UTM坐标，需要修改 `status_reporter.py` 中的 `gnss_pose_callback` 函数，添加坐标转换逻辑。

### 依赖检查

确保以下消息包已安装：
- `sensor_msgs` (ROS2标准包)
- `geometry_msgs` (ROS2标准包)
- `yunle_msgs` (车辆控制消息包)

如果 `yunle_msgs` 不可用，节点会发出警告但仍能运行，只是 `cur_speed` 将保持为0。

## 测试

### 发布测试数据

```bash
# 发布机器人状态
ros2 topic pub /robot_state std_msgs/String "data: 'running'" --once

# 发布GPS位置（示例）
ros2 topic pub /sensing/gnss/pose_with_covariance geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 116.397128, y: 39.916527, z: 50.5}}}}" --once

# 发布电池状态
ros2 topic pub /battery_status sensor_msgs/BatteryState \
  "{percentage: 0.85}" --once
```

### 查看MQTT消息

使用MQTT客户端订阅：

```bash
mosquitto_sub -h localhost -t "robot/status" -v
```

## 故障排除

### 问题1: 消息编译失败

**解决方案**:
```bash
# 清理构建
rm -rf build/ install/ log/
# 重新编译
colcon build --packages-select multi_map_navigation_msgs
```

### 问题2: GPS数据为0

**原因**: `/sensing/gnss/pose_with_covariance` 话题没有数据

**检查**:
```bash
ros2 topic list | grep gnss
ros2 topic echo /sensing/gnss/pose_with_covariance
```

### 问题3: 速度数据为0

**原因**: `/vehicle_status` 话题没有数据或 `yunle_msgs` 未安装

**检查**:
```bash
ros2 topic list | grep vehicle
ros2 topic info /vehicle_status
```

## 扩展

如果需要添加更多字段，修改以下文件：

1. `msg/RobotStatus.msg` - 添加新字段
2. `status_reporter.py` - 添加订阅和回调函数
3. 重新编译消息包

## 相关文件

- 消息定义: `multi_map_navigation_msgs/msg/RobotStatus.msg`
- 状态上报节点: `multi_map_navigation/status_reporter.py`
- 配置文件: `config/mqtt_config.yaml`
- 启动文件: `launch/multi_map_navigation.launch.py`
