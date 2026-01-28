# 项目修改总结

## 修改完成 ✅

根据新的JSON API接口格式，已完成以下所有修改工作：

### 1. 消息定义文件 (msg/)

#### ✅ Waypoint.msg
- 移除了 `geometry_msgs/PoseStamped pose` 字段
- 移除了 `string waypoint_type` 字段
- 移除了 `int32 waypoint_id` 字段
- **新增字段:**
  - `string name` - 航点名称
  - `int32 id` - 航点ID
  - `float64 lng, lat` - GPS坐标
  - `float64 x, y, yaw` - 当前地图坐标系下的位姿
  - `int32 type` - 航点类型（0=普通, 1=地图切换）
  - `float64 next_x, next_y, next_yaw` - 下一张地图坐标系下的位姿
  - `string next_map_name` - 下一张地图名称

#### ✅ WaypointList.msg
- 保持不变，仍然包含 `Waypoint[]` 数组

#### ✅ MapSwitchTrigger.msg
- 保持不变，无需修改

### 2. 源代码文件 (src/)

#### ✅ mqtt_task_receiver.py
**主要修改:**
- 移除了 `geometry_msgs.msg.PoseStamped` 导入
- 重写了 `parse_waypoint_list()` 方法
- 支持两种JSON输入格式：
  1. 直接数组格式 `[{...}, {...}]`
  2. 带task_id的格式 `{"task_id": "xxx", "waypoints": [...]}`
- 直接解析基本数据类型字段（x, y, yaw等）
- 根据 `type` 字段判断是否解析 `next_*` 字段
- 添加了字符串到浮点数的类型转换

**关键代码片段:**
```python
# 基本信息
waypoint.name = wp_data.get('name', '')
waypoint.id = wp_data.get('id', 0)
waypoint.map_name = wp_data.get('map_name', '')

# GPS坐标
waypoint.lng = wp_data.get('lng', 0.0)
waypoint.lat = wp_data.get('lat', 0.0)

# 当前地图坐标系下的位姿
waypoint.x = wp_data.get('x', 0.0)
waypoint.y = wp_data.get('y', 0.0)
waypoint.yaw = wp_data.get('yaw', 0.0)

# 航点类型
waypoint.type = wp_data.get('type', 0)

# 地图切换信息（仅当type为1时有效）
if waypoint.type == 1:
    waypoint.next_map_name = wp_data.get('next_map_name', '')
    waypoint.next_x = float(wp_data.get('next_x', 0.0))
    waypoint.next_y = float(wp_data.get('next_y', 0.0))
    waypoint.next_yaw = float(wp_data.get('next_yaw', 0.0))
```

#### ✅ navigation_manager.py
**主要修改:**
- 修改了 `is_map_switch_point()` 方法
  ```python
  # 旧: return waypoint.waypoint_type == 'map_switch'
  # 新: return waypoint.type == 1
  ```

- 重写了 `trigger_map_switch()` 方法
  - 使用 `waypoint.next_x, next_y, next_yaw` 构建切换位姿
  - 使用 `tf_transformations.quaternion_from_euler()` 转换yaw角
  - 使用 `waypoint.id` 替代 `waypoint.waypoint_id`

- 重写了 `send_nav2_goal()` 方法
  - 从基本字段构建 `PoseStamped` 消息
  - 使用 `waypoint.x, y, yaw` 构建位姿
  - 使用 `tf_transformations.quaternion_from_euler()` 转换yaw角

- 简化了 `on_goal_reached()` 方法
  - 移除了对 `waypoint_type == 'final'` 的判断
  - 只检查是否为最后一个航点

**关键代码片段:**
```python
# 构建位姿
pose = PoseStamped()
pose.header.stamp = self.get_clock().now().to_msg()
pose.header.frame_id = waypoint.map_name
pose.pose.position.x = waypoint.x
pose.pose.position.y = waypoint.y
pose.pose.position.z = 0.0

# 将yaw角转换为四元数
q = quaternion_from_euler(0, 0, waypoint.yaw)
pose.pose.orientation.x = q[0]
pose.pose.orientation.y = q[1]
pose.pose.orientation.z = q[2]
pose.pose.orientation.w = q[3]
```

#### ✅ map_switch_controller.py
- **无需修改** - 该文件只处理 `MapSwitchTrigger` 消息，消息格式未变

#### ✅ process_manager.py
- **无需修改** - 该文件只管理进程生命周期，与消息格式无关

#### ✅ status_reporter.py
- **无需修改** - 该文件只发布状态信息，与航点格式无关

#### ✅ main.py
- **无需修改** - 主入口文件，无需改动

### 3. 配置文件 (config/)

#### ✅ mqtt_config.yaml
- 保持不变，无需修改

#### ✅ navigation_config.yaml
- 保持不变，无需修改

### 4. 文档文件

#### ✅ README.md
- 更新了 MQTT消息格式示例
- 添加了新格式的字段说明
- 更新了两种支持的JSON格式示例

#### ✅ CHANGELOG_API.md (新增)
- 详细的API变更说明文档
- 字段对照表
- 代码修改详情
- 迁移步骤
- 测试建议
- 已知问题说明

#### ✅ QUICK_REFERENCE.md (新增)
- 快速参考指南
- 新旧API对比
- JSON模板示例
- 常见问题解答
- 测试命令
- Python客户端示例代码
- 调试技巧

### 5. 启动文件 (launch/)

#### ✅ multi_map_navigation.launch.py
- 保持不变，无需修改

## 新增依赖

### Python包
- `tf_transformations` - 用于yaw角到四元数的转换

**安装方法:**
```bash
pip3 install transforms3d
# 或
sudo apt-get install ros-humble-tf-transformations
```

## 编译和部署

### 1. 重新编译ROS2包
```bash
cd ~/workspace/Car_jetson
colcon build --packages-select multi_map_navigation
source install/setup.bash
```

### 2. 验证编译
```bash
ros2 interface show multi_map_navigation/msg/Waypoint
```

### 3. 启动系统
```bash
ros2 launch multi_map_navigation multi_map_navigation.launch.py
```

## 测试验证

### 1. 测试单地图导航
```bash
mosquitto_pub -h localhost -t robot/task -m '[
  {
    "name": "test_point",
    "lng": 116.397428,
    "lat": 39.909230,
    "x": 1.0,
    "y": 2.0,
    "yaw": 0.0,
    "id": 1,
    "map_name": "map1",
    "type": 0
  }
]'
```

### 2. 测试地图切换
```bash
mosquitto_pub -h localhost -t robot/task -m '[
  {
    "name": "point1",
    "lng": 116.397428,
    "lat": 39.909230,
    "x": 1.0,
    "y": 2.0,
    "yaw": 0.0,
    "id": 1,
    "map_name": "map1",
    "type": 0
  },
  {
    "name": "switch_point",
    "lng": 116.397500,
    "lat": 39.909300,
    "x": 5.0,
    "y": 3.0,
    "yaw": 1.57,
    "id": 2,
    "map_name": "map1",
    "next_map_name": "map2",
    "next_x": 0.5,
    "next_y": 0.5,
    "next_yaw": 0.0,
    "type": 1
  }
]'
```

### 3. 监控系统状态
```bash
# 监控航点列表
ros2 topic echo /waypoint_list

# 监控机器人状态
ros2 topic echo /robot_state

# 监控地图切换
ros2 topic echo /trigger_map_switch
ros2 topic echo /map_switch_complete
```

## 关键变更点总结

### 数据结构变更
| 项目 | 旧格式 | 新格式 |
|------|--------|--------|
| 航点ID | `waypoint_id` (int32) | `id` (int32) |
| 航点类型 | `waypoint_type` (string) | `type` (int32: 0或1) |
| 位姿存储 | `pose` (PoseStamped) | `x, y, yaw` (float64) |
| GPS坐标 | 无 | `lng, lat` (float64) |
| 航点名称 | 无 | `name` (string) |
| 下一地图坐标 | 无 | `next_x, next_y, next_yaw` (float64) |

### 类型判断变更
```python
# 旧方式
if waypoint.waypoint_type == 'map_switch':
    # 地图切换点
elif waypoint.waypoint_type == 'normal':
    # 普通点
elif waypoint.waypoint_type == 'final':
    # 最终点

# 新方式
if waypoint.type == 1:
    # 地图切换点
else:  # waypoint.type == 0
    # 普通点
# 最后一个点自动识别为终点
```

### 坐标获取变更
```python
# 旧方式
x = waypoint.pose.pose.position.x
y = waypoint.pose.pose.position.y
# 需要从四元数转换yaw

# 新方式
x = waypoint.x
y = waypoint.y
yaw = waypoint.yaw  # 直接获取弧度值
```

## 兼容性说明

### ❌ 不向后兼容
- 旧的JSON格式无法使用
- 需要重新编译ROS2消息
- 需要更新所有MQTT客户端

### ✅ 功能保持不变
- 导航逻辑完全相同
- 地图切换流程不变
- 进程管理机制不变
- 状态报告功能不变

## 注意事项

1. **角度单位**: 所有yaw角度必须使用**弧度制**，不是角度制
2. **类型安全**: JSON中的next_x等字段可能是字符串，代码已做类型转换
3. **必填字段**: name, lng, lat, x, y, yaw, id, map_name, type 都是必填的
4. **地图切换**: type=1时，next_map_name, next_x, next_y, next_yaw 必须提供
5. **坐标系**: next_x, next_y, next_yaw 是在**下一张地图坐标系**下的坐标

## 文件清单

### 已修改的文件
- ✅ `msg/Waypoint.msg`
- ✅ `src/communication/mqtt_task_receiver.py`
- ✅ `src/navigation/navigation_manager.py`
- ✅ `README.md`

### 新增的文件
- ✅ `CHANGELOG_API.md` - API变更详细说明
- ✅ `QUICK_REFERENCE.md` - 快速参考指南
- ✅ `SUMMARY.md` - 本文件

### 无需修改的文件
- ✅ `msg/WaypointList.msg`
- ✅ `msg/MapSwitchTrigger.msg`
- ✅ `src/communication/status_reporter.py`
- ✅ `src/navigation/map_switch_controller.py`
- ✅ `src/navigation/process_manager.py`
- ✅ `src/main.py`
- ✅ `config/mqtt_config.yaml`
- ✅ `config/navigation_config.yaml`
- ✅ `launch/multi_map_navigation.launch.py`

## 下一步操作

1. **编译项目**
   ```bash
   cd ~/workspace/Car_jetson
   colcon build --packages-select multi_map_navigation
   source install/setup.bash
   ```

2. **安装依赖**
   ```bash
   pip3 install transforms3d
   ```

3. **测试验证**
   - 使用提供的测试命令验证功能
   - 检查日志确认无错误
   - 测试单地图和多地图场景

4. **更新客户端**
   - 修改MQTT发送端代码
   - 使用新的JSON格式
   - 确保type字段为整数

5. **生产部署**
   - 在测试环境充分验证后再部署到生产环境
   - 准备回滚方案（保留旧版本代码）

## 技术支持

如有问题，请参考：
1. `CHANGELOG_API.md` - 详细的变更说明
2. `QUICK_REFERENCE.md` - 快速参考和示例
3. `README.md` - 完整的项目文档

或查看日志：
```bash
ros2 run multi_map_navigation mqtt_task_receiver.py
```