# Multi-Map Navigation Project Architecture

## 项目概述
多地图导航系统，支持MQTT通信、动态地图切换、实时状态上报。

## 模块划分（两人协作）

### 👤 开发者A：通信模块 (Communication Module)
负责功能1和功能4：MQTT通信、任务接收、状态上报

### 👤 开发者B：导航控制模块 (Navigation Control Module)
负责功能2和功能3：路点导航、地图切换、进程管理

---

## 详细架构设计

### 1. 通信模块 (Developer A)

#### 1.1 MQTT任务接收器 (mqtt_task_receiver)
**文件**: `src/communication/mqtt_task_receiver.py`

**职责**:
- 连接MQTT Broker
- 订阅任务主题，接收路点序列消息
- 解析路点数据（地图名称、坐标、位姿、类型等）
- 将路点列表发布到ROS2话题 `/waypoint_list`

**关键接口**:
```python
class MQTTTaskReceiver:
    def connect_broker(broker_url, port)
    def on_message_callback(topic, payload)
    def parse_waypoint_list(json_data) -> List[Waypoint]
    def publish_to_ros(waypoint_list)
```

**输出话题**:
- `/waypoint_list` (custom_msgs/WaypointList)

---

#### 1.2 状态上报器 (status_reporter)
**文件**: `src/communication/status_reporter.py`

**职责**:
- 订阅机器人状态相关ROS2话题
- 聚合状态信息（GPS、电量、速度、工作状态）
- 按心跳频率（默认3秒）发送到MQTT Broker

**关键接口**:
```python
class StatusReporter:
    def __init__(heartbeat_rate=3.0)
    def subscribe_robot_topics()
    def aggregate_status() -> RobotStatus
    def publish_to_mqtt(status)
```

**订阅话题**:
- `/gps/fix` (sensor_msgs/NavSatFix)
- `/battery_state` (sensor_msgs/BatteryState)
- `/cmd_vel` (geometry_msgs/Twist)
- `/robot_state` (std_msgs/String) - idle/running

**MQTT发布主题**:
- `robot/status` (JSON格式心跳包)

---

### 2. 导航控制模块 (Developer B)

#### 2.1 导航管理器 (navigation_manager)
**文件**: `src/navigation/navigation_manager.py`

**职责**:
- 订阅 `/waypoint_list` 获取路点序列
- 管理当前导航状态（当前路点索引、地图名称）
- 顺序发送导航目标到Navigation2
- 检测路点到达事件
- 识别地图切换点，触发地图切换流程

**关键接口**:
```python
class NavigationManager:
    def load_waypoint_list(waypoint_list)
    def navigate_to_next_waypoint()
    def check_goal_reached() -> bool
    def is_map_switch_point(waypoint) -> bool
    def send_nav2_goal(waypoint)
```

**订阅话题**:
- `/waypoint_list` (custom_msgs/WaypointList)
- `/goal_reached` (action feedback from Nav2)

**发布话题**:
- `/robot_state` (std_msgs/String) - idle/running
- `/trigger_map_switch` (custom_msgs/MapSwitchTrigger)

---

#### 2.2 地图切换控制器 (map_switch_controller)
**文件**: `src/navigation/map_switch_controller.py`

**职责**:
- 监听地图切换触发信号
- 关闭当前进程（re_localization, lio_sam, navigation2）
- 启动新地图的进程（re_localization, lio_sam, navigation2）
- 确保进程干净关闭和启动
- 通知导航管理器切换完成

**关键接口**:
```python
class MapSwitchController:
    def shutdown_current_stack()
    def wait_for_clean_shutdown(timeout=10.0)
    def launch_new_stack(map_name)
    def verify_stack_ready() -> bool
```

**订阅话题**:
- `/trigger_map_switch` (custom_msgs/MapSwitchTrigger)

**发布话题**:
- `/map_switch_complete` (std_msgs/Bool)

---

#### 2.3 进程管理器 (process_manager)
**文件**: `src/navigation/process_manager.py`

**职责**:
- 封装ROS2 launch文件的启动/关闭
- 管理进程生命周期（re_localization, lio_sam, navigation2）
- 提供进程状态查询接口

**关键接口**:
```python
class ProcessManager:
    def launch_relocalization(map_name)
    def launch_liosam()
    def launch_navigation2(map_name)
    def shutdown_process(process_name)
    def is_process_running(process_name) -> bool
```

---

## 数据流图

```
MQTT Broker
    ↓ (waypoint list)
mqtt_task_receiver
    ↓ /waypoint_list
navigation_manager
    ↓ /trigger_map_switch
map_switch_controller
    ↓ launch/shutdown
process_manager
    ↓ ROS2 Launch
[re_localization, lio_sam, navigation2]
    ↓ /goal_reached
navigation_manager
    ↓ /robot_state
status_reporter
    ↓ (heartbeat)
MQTT Broker
```

---

## 自定义消息定义

### WaypointList.msg
```
Header header
Waypoint[] waypoints
```

### Waypoint.msg
```
string map_name
geometry_msgs/PoseStamped pose
string waypoint_type  # normal, map_switch, final
string next_map_name  # 仅在map_switch类型时有效
```

### MapSwitchTrigger.msg
```
string current_map
string next_map
geometry_msgs/PoseStamped switch_pose
```

---

## 配置文件

### config/mqtt_config.yaml
```yaml
mqtt:
  broker_url: "mqtt.example.com"
  port: 1883
  username: "robot"
  password: "password"
  task_topic: "robot/task"
  status_topic: "robot/status"

heartbeat:
  rate: 3.0  # seconds
```

### config/navigation_config.yaml
```yaml
navigation:
  goal_tolerance: 0.5  # meters
  map_switch_timeout: 30.0  # seconds

launch_files:
  relocalization: "re_localization"
  liosam: "lio_sam"
  navigation2: "navigation2"
```

---

## 项目目录结构

```
multi_map_navigation/
├── src/
│   ├── communication/              # 开发者A
│   │   ├── mqtt_task_receiver.py
│   │   └── status_reporter.py
│   ├── navigation/                 # 开发者B
│   │   ├── navigation_manager.py
│   │   ├── map_switch_controller.py
│   │   └── process_manager.py
│   └── main.py                     # 主入口
├── msg/
│   ├── WaypointList.msg
│   ├── Waypoint.msg
│   └── MapSwitchTrigger.msg
├── config/
│   ├── mqtt_config.yaml
│   └── navigation_config.yaml
├── launch/
│   └── multi_map_navigation.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 开发分工建议

### 开发者A任务清单
1. 实现MQTT连接和消息解析
2. 定义Waypoint相关消息格式
3. 实现mqtt_task_receiver节点
4. 实现status_reporter节点
5. 编写MQTT通信单元测试
6. 配置mqtt_config.yaml

### 开发者B任务清单
1. 实现process_manager进程管理
2. 实现map_switch_controller地图切换逻辑
3. 实现navigation_manager导航状态机
4. 集成Navigation2 Action Client
5. 编写导航逻辑单元测试
6. 配置navigation_config.yaml

---

## 接口约定（两人协作关键）

### 1. 话题接口
- `/waypoint_list`: A发布 → B订阅
- `/robot_state`: B发布 → A订阅

### 2. 消息格式
- 共同定义msg/目录下的消息格式
- 使用Git协作，消息定义变更需双方确认

### 3. 测试策略
- A可以用模拟MQTT Broker测试通信模块
- B可以用rosbag或手动发布 `/waypoint_list` 测试导航模块
- 集成测试时双方联调

---

## 开发流程建议

### Phase 1: 接口定义（1天）
- 共同定义消息格式
- 确定话题名称和数据结构
- 创建项目骨架

### Phase 2: 并行开发（5-7天）
- A: 开发通信模块
- B: 开发导航模块
- 每日同步进度

### Phase 3: 集成测试（2-3天）
- 联调MQTT → 导航完整流程
- 测试地图切换场景
- 压力测试和异常处理

### Phase 4: 优化部署（1-2天）
- 性能优化
- 日志完善
- 部署文档

---

## 关键技术点

### 通信模块
- 使用paho-mqtt库
- 实现断线重连机制
- JSON消息序列化

### 导航模块
- Navigation2 Action Client
- subprocess管理ROS2 launch
- 状态机设计（idle → running → switching → running → idle）

### 进程管理
- 使用ros2 launch API或subprocess
- 优雅关闭（SIGTERM → 等待 → SIGKILL）
- 进程健康检查

---

## 注意事项

1. **线程安全**: MQTT回调和ROS2回调可能在不同线程，注意加锁
2. **异常处理**: 网络断开、进程崩溃、导航失败等场景
3. **日志记录**: 使用ROS2 logger，便于调试
4. **参数化配置**: 避免硬编码，使用yaml配置文件
5. **版本控制**: 使用Git分支开发，定期合并

---

## 扩展性考虑

- 支持多机器人（通过robot_id区分）
- 支持任务优先级和中断
- 支持路点动态更新
- 支持地图预加载优化切换速度
