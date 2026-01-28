# API接口变更说明

## 概述

本次更新根据新的JSON数据格式修改了项目中的相关命名和逻辑，原有功能逻辑保持不变。

## 主要变更

### 1. 消息定义变更

#### Waypoint.msg

**旧格式:**
- 使用 `geometry_msgs/PoseStamped pose` 存储位姿
- 使用 `string waypoint_type` 表示类型（"normal", "map_switch", "final"）
- 使用 `int32 waypoint_id` 作为标识符

**新格式:**
- 直接使用基本数据类型存储坐标和角度
- 使用 `int32 type` 表示类型（0=正常导航点, 1=地图切换点）
- 使用 `int32 id` 作为标识符
- 新增 `string name` 航点名称
- 新增 `float64 lng, lat` GPS坐标
- 新增 `float64 x, y, yaw` 当前地图坐标系下的位姿
- 新增 `float64 next_x, next_y, next_yaw` 下一张地图坐标系下的位姿

**字段对照表:**

| 旧字段 | 新字段 | 说明 |
|--------|--------|------|
| waypoint_id | id | 航点标识符 |
| waypoint_type | type | 类型改为整数（0或1） |
| pose.pose.position.x | x | 直接存储X坐标 |
| pose.pose.position.y | y | 直接存储Y坐标 |
| pose.pose.orientation | yaw | 改为直接存储偏航角（弧度） |
| - | name | 新增：航点名称 |
| - | lng, lat | 新增：GPS坐标 |
| - | next_x, next_y, next_yaw | 新增：下一张地图坐标系下的位姿 |

### 2. MQTT消息格式变更

#### 接收的任务消息

**支持两种格式:**

**格式1: 直接数组**
```json
[
  {
    "name": "point1",
    "lng": 116.397428,
    "lat": 39.90923,
    "x": 1.0,
    "y": 2.0,
    "yaw": 0.0,
    "id": 1,
    "map_name": "map1",
    "type": 0
  }
]
```

**格式2: 带任务ID**
```json
{
  "task_id": "task_001",
  "waypoints": [...]
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| name | string | 是 | 航点名称 |
| id | integer | 是 | 航点唯一标识符 |
| lng | number | 是 | 经度 |
| lat | number | 是 | 纬度 |
| x | number | 是 | 当前地图坐标系下的X坐标 |
| y | number | 是 | 当前地图坐标系下的Y坐标 |
| yaw | number | 是 | 当前地图坐标系下的偏航角（弧度） |
| map_name | string | 是 | 当前地图名称 |
| type | integer | 是 | 航点类型（0=正常, 1=地图切换） |
| next_map_name | string | type=1时必填 | 下一张地图名称 |
| next_x | number | type=1时必填 | 下一张地图坐标系下的X坐标 |
| next_y | number | type=1时必填 | 下一张地图坐标系下的Y坐标 |
| next_yaw | number | type=1时必填 | 下一张地图坐标系下的偏航角 |
| tolerance | number | 否 | 目标容差（米），默认0.5 |

### 3. 代码修改详情

#### 3.1 mqtt_task_receiver.py

**主要变更:**
- 移除了 `geometry_msgs.msg.PoseStamped` 的导入
- 修改 `parse_waypoint_list()` 方法以支持新的JSON格式
- 支持两种输入格式（直接数组或带task_id的对象）
- 直接解析基本数据类型字段（x, y, yaw等）
- 根据type字段判断是否解析next_*字段

**关键代码变更:**
```python
# 旧代码
waypoint.waypoint_id = wp_data.get('waypoint_id', 0)
waypoint.waypoint_type = wp_data.get('waypoint_type', 'normal')
waypoint.pose = PoseStamped()
waypoint.pose.pose.position.x = position.get('x', 0.0)

# 新代码
waypoint.id = wp_data.get('id', 0)
waypoint.type = wp_data.get('type', 0)
waypoint.x = wp_data.get('x', 0.0)
waypoint.y = wp_data.get('y', 0.0)
waypoint.yaw = wp_data.get('yaw', 0.0)
```

#### 3.2 navigation_manager.py

**主要变更:**
- 修改 `is_map_switch_point()` 方法的判断逻辑
- 修改 `trigger_map_switch()` 方法以使用新的坐标字段
- 修改 `send_nav2_goal()` 方法以从基本字段构建PoseStamped
- 移除对 `waypoint_type == 'final'` 的判断

**关键代码变更:**
```python
# 旧代码
def is_map_switch_point(self, waypoint: Waypoint) -> bool:
    return waypoint.waypoint_type == 'map_switch'

# 新代码
def is_map_switch_point(self, waypoint: Waypoint) -> bool:
    return waypoint.type == 1
```

```python
# 旧代码
goal_msg.pose = waypoint.pose

# 新代码
pose = PoseStamped()
pose.pose.position.x = waypoint.x
pose.pose.position.y = waypoint.y
q = quaternion_from_euler(0, 0, waypoint.yaw)
pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
goal_msg.pose = pose
```

#### 3.3 map_switch_controller.py

**说明:**
- 该文件无需修改，因为它只处理MapSwitchTrigger消息
- MapSwitchTrigger消息的格式保持不变

### 4. 依赖变更

**新增依赖:**
- `tf_transformations` - 用于yaw角到四元数的转换

**安装方法:**
```bash
pip3 install transforms3d
# 或
sudo apt-get install ros-humble-tf-transformations
```

### 5. 兼容性说明

#### 向后兼容性
- ❌ 不兼容旧的JSON格式
- ❌ 需要重新编译ROS2消息定义

#### 迁移步骤

1. **更新消息定义**
```bash
cd ~/workspace/Car_jetson
colcon build --packages-select multi_map_navigation
source install/setup.bash
```

2. **更新MQTT客户端**
   - 修改发送端以使用新的JSON格式
   - 确保type字段为整数类型
   - 地图切换点必须包含next_*字段

3. **测试验证**
```bash
# 发送测试消息
mosquitto_pub -h localhost -t robot/task -m '[
  {
    "name": "test_point",
    "lng": 116.397428,
    "lat": 39.90923,
    "x": 1.0,
    "y": 2.0,
    "yaw": 0.0,
    "id": 1,
    "map_name": "map1",
    "type": 0
  }
]'
```

### 6. 测试建议

#### 单元测试
- [ ] 测试正常导航点解析
- [ ] 测试地图切换点解析
- [ ] 测试两种JSON格式的兼容性
- [ ] 测试缺失可选字段的处理

#### 集成测试
- [ ] 测试单地图导航
- [ ] 测试多地图切换
- [ ] 测试GPS坐标记录
- [ ] 测试yaw角转换正确性

### 7. 已知问题

1. **坐标系转换**
   - 需要确保next_x, next_y, next_yaw是在下一张地图坐标系下的正确坐标
   - 建议在地图切换点进行坐标系对齐验证

2. **角度单位**
   - 所有yaw角度使用弧度制
   - 需要确保上游系统发送的是弧度而非角度

3. **类型安全**
   - next_x, next_y, next_yaw在JSON中可能是字符串类型
   - 代码中已添加 `float()` 转换以确保类型正确

### 8. 性能影响

- ✅ 减少了消息大小（不再使用嵌套的pose结构）
- ✅ 简化了解析逻辑
- ⚠️ 需要在运行时进行yaw到四元数的转换（性能影响可忽略）

### 9. 文档更新

已更新以下文档：
- ✅ README.md - MQTT消息格式示例
- ✅ msg/Waypoint.msg - 消息定义注释
- ✅ msg/WaypointList.msg - 消息定义注释
- ✅ msg/MapSwitchTrigger.msg - 消息定义注释

### 10. 示例代码

#### Python客户端发送示例

```python
import paho.mqtt.client as mqtt
import json

# 创建任务
task = [
    {
        "name": "起点",
        "lng": 116.397428,
        "lat": 39.90923,
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
        "id": 1,
        "map_name": "floor1",
        "type": 0
    },
    {
        "name": "地图切换点",
        "lng": 116.397500,
        "lat": 39.90930,
        "x": 10.0,
        "y": 5.0,
        "yaw": 1.57,
        "id": 2,
        "map_name": "floor1",
        "next_map_name": "floor2",
        "next_x": 0.5,
        "next_y": 0.5,
        "next_yaw": 0.0,
        "type": 1
    },
    {
        "name": "终点",
        "lng": 116.397600,
        "lat": 39.90940,
        "x": 15.0,
        "y": 10.0,
        "yaw": 0.0,
        "id": 3,
        "map_name": "floor2",
        "type": 0
    }
]

# 发送到MQTT
client = mqtt.Client()
client.connect("localhost", 1883, 60)
client.publish("robot/task", json.dumps(task))
client.disconnect()
```

## 总结

本次更新主要是为了适配新的API接口格式，核心导航逻辑保持不变。主要改进包括：

1. ✅ 简化了数据结构，使用基本类型替代复杂的嵌套消息
2. ✅ 增加了GPS坐标支持，便于轨迹记录和分析
3. ✅ 明确了地图切换点的坐标表示方式
4. ✅ 支持更灵活的JSON输入格式
5. ✅ 保持了原有的功能和性能

建议在生产环境部署前进行充分的测试验证。
