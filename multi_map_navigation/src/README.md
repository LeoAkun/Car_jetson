# 多地图导航系统

基于ROS2的多地图导航系统，具有MQTT通信和动态地图切换功能。

## 概述

该系统使机器人能够无缝地在多个地图中导航，通过MQTT接收航点任务并实时报告状态。它处理自动地图切换、进程管理和导航协调。

## 功能特性

- **MQTT通信**: 通过MQTT接收导航任务并发布机器人状态
- **多地图导航**: 在不同地图的航点之间导航
- **图路径规划**: 基于NetworkX的多路径规划，支持导航失败时自动切换备用路径
- **动态地图切换**: 在导航过程中自动切换地图
- **进程管理**: 管理导航堆栈组件的生命周期
- **实时状态报告**: 基于心跳的状态更新（GPS、电池、速度、状态）
- **模块化架构**: 通信和导航模块之间的清晰分离

## 架构

### 模块概览

```
                        ┌───────────────────┐
                        │     MQTT 代理      │
                        └──┬─────────────▲──┘
                           │             │
            订阅 .../task/start    发布 .../vehicle_status
                           │             │
┌──────────────────────────┼─────────────┼───────────────────────────────────┐
│  multi_map_navigation    │             │                                   │
│                          ▼             │                                   │
│  ┌────────────────────────┐   ┌───────┴──────────────┐  /sensing/gnss/... │
│  │  mqtt_task_receiver    │   │  status_reporter     │◄ /battery_status   │
│  │  (MQTT → ROS2)         │   │  (ROS2 → MQTT)       │◄ /vehicle_status   │
│  └──────────┬─────────────┘   └──────────▲───────────┘                    │
│             │ /StartEndGraph             │                                │
│             │                            │ /robot_state                   │
│  ┌──────────┼────────────────────┐       │                                │
│  │          │  /start_end_graph  │       │                                │
│  │          ▼                    │       │                                │
│  │  ┌────────────────────────┐   │       │                                │
│  │  │  route_planner         │   │       │                                │
│  │  │  - NetworkX 图路径规划  │   │       │                                │
│  │  │  - 多条备用路径计算     │   │       │                                │
│  │  │  - srv: pub_new_path   │   │       │                                │
│  │  └──────────┬─────────────┘   │       │                                │
│  │             │ /waypoint_list   │       │                                │
│  └─────────────┼─────────────────┘       │                                │
│                ▼                         │                                │
│  ┌───────────────────────────────────────┴──────────────────────┐         │
│  │  navigation_manager                                          │         │
│  │  - 接收航点列表，按序导航                                     │         │
│  │  - 航点类型: 1=普通  2=红绿灯  3=充电  4=地图切换             │         │
│  │  - Nav2 动作客户端 (/navigate_to_pose)                       │         │
│  │  - 导航失败时调用 route_planner 获取备用路径                   │         │
│  │  - 首次导航时通过 srv 启动完整导航堆栈                        │         │
│  └──────┬──────────────────────────────────▲─────────────────────┘         │
│         │ /trigger_map_switch              │ /map_switch_complete          │
│         ▼                                  │                              │
│  ┌─────────────────────────────────────────┴───────────────────┐          │
│  │  map_switch_controller                                      │          │
│  │  4步流程: 关闭堆栈 → 等待清理 → 启动新堆栈 → 验证就绪       │          │
│  └──────────────────────┬──────────────────────────────────────┘          │
│                         │ srv 调用 (navigation_manager 也直接调用)         │
│                         ▼                                                 │
│  ┌─────────────────────────────────────────────────────────────┐          │
│  │  process_manager                                            │          │
│  │  srv: start_process / shutdown_process / get_status         │          │
│  └──────────────────────┬──────────────────────────────────────┘          │
│                         │ subprocess 启动/关闭                             │
└─────────────────────────┼─────────────────────────────────────────────────┘
                          ▼
            ┌──────────────────────────────────┐
            │        外部 ROS2 进程              │
            │                                  │
            │  启动: re_localization            │
            │     → nav2_init_pose             │
            │     → (等待 map→odom TF)         │
            │     → lio_sam                    │
            │     → navigation2                │
            │                                  │
            │  关闭: navigation2 → lio_sam     │
            │     → nav2_init_pose             │
            │     → re_localization            │
            └──────────────────────────────────┘
```

### 节点间通信

| 来源 | 目标 | 通道 | 类型 |
|------|------|------|------|
| mqtt_task_receiver | navigation_manager | `/waypoint_list` | Topic (WaypointList) |
| route_planner | navigation_manager | `/waypoint_list` | Topic (WaypointList) |
| 外部（MQTT等） | route_planner | `/start_end_graph` | Topic (StartEndGraph) |
| navigation_manager | route_planner | `/route_planner/pub_new_path` | Service (PubNewPath) |
| navigation_manager | status_reporter | `/robot_state` | Topic (String) |
| navigation_manager | map_switch_controller | `/trigger_map_switch` | Topic (MapSwitchTrigger) |
| map_switch_controller | navigation_manager | `/map_switch_complete` | Topic (Bool) |
| navigation_manager | Nav2 | `/navigate_to_pose` | Action (NavigateToPose) |
| navigation_manager | process_manager | `start_process` / `shutdown_process` | Service |
| map_switch_controller | process_manager | `start_process` / `shutdown_process` / `get_status` | Service |
| 传感器 | status_reporter | `/sensing/gnss/...`, `/battery_status`, `/vehicle_status` | Topic |

## 项目结构

```
src/
├── multi_map_navigation/                  # 主功能包 (ament_python)
│   ├── multi_map_navigation/              # Python 模块
│   │   ├── __init__.py
│   │   ├── navigation_manager.py          # 导航管理器（航点导航、地图切换检测）
│   │   ├── map_switch_controller.py       # 地图切换控制器（协调切换流程）
│   │   ├── process_manager.py             # 进程管理器（管理导航堆栈生命周期）
│   │   ├── mqtt_task_receiver.py          # MQTT 任务接收（解析航点JSON）
│   │   ├── status_reporter.py             # 状态上报（聚合状态发送至MQTT）
│   │   └── route_planner.py              # 图路径规划（NetworkX多路径规划与备用路径切换）
│   ├── config/
│   │   ├── navigation_config.yaml         # 导航参数配置
│   │   └── mqtt_config.yaml               # MQTT 连接配置
│   ├── launch/
│   │   └── multi_map_navigation.launch.py # 启动所有节点
│   ├── test/                              # 测试文件
│   │   ├── test_navigation_manager.py
│   │   ├── test_map_switch_controller.py
│   │   ├── test_route_planner.py
│   │   ├── test_copyright.py
│   │   ├── test_flake8.py
│   │   └── test_pep257.py
│   ├── resource/
│   ├── setup.py                           # 入口点定义
│   ├── setup.cfg
│   └── package.xml
├── multi_map_navigation_msgs/             # 消息/服务定义包 (ament_cmake)
│   ├── msg/
│   │   ├── Waypoint.msg                   # 单个航点定义
│   │   ├── WaypointList.msg               # 航点列表（含任务元数据）
│   │   ├── MapSwitchTrigger.msg           # 地图切换触发
│   │   ├── RobotStatus.msg                # 机器人综合状态（用于MQTT上报）
│   │   ├── Edge.msg                       # 图的边（起止节点ID + 权重）
│   │   └── StartEndGraph.msg              # 图结构（起止点 + 节点列表 + 边列表）
│   ├── srv/
│   │   ├── StartProcess.srv               # 启动导航进程
│   │   ├── ShutdownProcess.srv            # 关闭导航进程
│   │   ├── GetProcessStatus.srv           # 查询所有进程状态
│   │   └── PubNewPath.srv                 # 请求发布备用路径
│   ├── CMakeLists.txt
│   └── package.xml
├── map/                                   # 地图数据
│   ├── map_network.yaml                   # 地图网络图结构定义
│   ├── map1/
│   │   ├── map1_clean.pgm                 # 栅格地图
│   │   ├── map1_clean.yaml                # 地图元数据
│   │   └── map1.csv                       # GPS航点数据
│   └── map2/
│       ├── map2_clean.pgm
│       ├── map2_clean.yaml
│       └── map2.csv
└── README.md
```

## 安装

### 前置条件

- ROS2 (Humble或更高版本)
- Python 3.8+
- paho-mqtt
- Navigation2
- LIO-SAM
- re_localization包

### 安装依赖

```bash
# 安装Python依赖
pip3 install paho-mqtt

# 安装ROS2依赖
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2
```

### 构建包

```bash
cd ~/workspace/Car_jetson
colcon build --packages-select multi_map_navigation_msgs multi_map_navigation
source install/setup.bash
```

## 配置

### MQTT配置

编辑 `config/mqtt_config.yaml`:

```yaml
mqtt:
  broker_url: "your_mqtt_broker.com"
  port: 1883
  username: "your_username"
  password: "your_password"
  task_topic: "robot/task"
  status_topic: "robot/status"

heartbeat:
  rate: 3.0  # 秒
```

### 导航配置

编辑 `config/navigation_config.yaml`:

```yaml
navigation:
  goal_tolerance: 0.5
  navigation_timeout: 300.0
  map_switch_timeout: 30.0

maps:
  map_directory: "/home/akun/maps"
```

## 使用方法

### 启动所有节点

```bash
ros2 launch multi_map_navigation multi_map_navigation.launch.py
```

### 启动单个节点

```bash
# 通信模块
ros2 run multi_map_navigation mqtt_task_receiver
ros2 run multi_map_navigation status_reporter

# 导航模块
ros2 run multi_map_navigation navigation_manager
ros2 run multi_map_navigation map_switch_controller
ros2 run multi_map_navigation process_manager

# 路径规划模块
ros2 run multi_map_navigation route_planner
```

## MQTT消息格式

### 任务消息（接收）

主题: `robot/task`

支持两种格式：

**格式1: 直接数组格式**
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
    "type": 1
  },
  {
    "name": "switch_point",
    "lng": 116.397500,
    "lat": 39.90930,
    "x": 5.0,
    "y": 3.0,
    "yaw": 1.57,
    "id": 2,
    "map_name": "map1",
    "next_map_name": "map2",
    "next_x": 0.5,
    "next_y": 0.5,
    "next_yaw": 0.0,
    "type": 4
  },
  {
    "name": "point3",
    "lng": 116.397600,
    "lat": 39.90940,
    "x": 10.0,
    "y": 8.0,
    "yaw": 0.0,
    "id": 3,
    "map_name": "map2",
    "type": 1
  }
]
```

**格式2: 带任务ID的格式**
```json
{
  "task_id": "task_001",
  "waypoints": [
    {
      "name": "point1",
      "lng": 116.397428,
      "lat": 39.90923,
      "x": 1.0,
      "y": 2.0,
      "yaw": 0.0,
      "id": 1,
      "map_name": "map1",
      "type": 1
    },
    {
      "name": "switch_point",
      "lng": 116.397500,
      "lat": 39.90930,
      "x": 5.0,
      "y": 3.0,
      "yaw": 1.57,
      "id": 2,
      "map_name": "map1",
      "next_map_name": "map2",
      "next_x": 0.5,
      "next_y": 0.5,
      "next_yaw": 0.0,
      "type": 4
    }
  ]
}
```

**字段说明:**
- `name`: 航点名称
- `lng`: 经度
- `lat`: 纬度
- `x`: 当前地图坐标系下的X坐标
- `y`: 当前地图坐标系下的Y坐标
- `yaw`: 当前地图坐标系下的偏航角（弧度）
- `id`: 航点唯一标识符
- `map_name`: 当前地图名称
- `type`: 航点类型（1=普通导航点, 2=红绿灯, 3=充电桩, 4=地图切换点）
- `next_map_name`: 下一张地图名称（仅type=4时需要）
- `next_x`: 当前点在下一张地图坐标系下的X坐标（仅type=4时需要）
- `next_y`: 当前点在下一张地图坐标系下的Y坐标（仅type=4时需要）
- `next_yaw`: 当前点在下一张地图坐标系下的偏航角（仅type=4时需要）

### 状态消息（发送）

主题: `prod/data/vehicle/{vin}/vehicle_status`

```json
{
  "vin": "LS1234567890",
  "task_status": 1,
  "gps_lng": 116.397428,
  "gps_lat": 39.90923,
  "gps_alt": 100.0,
  "cur_speed": 0.5,
  "battery_capacity": 85.5
}
```

## 自定义消息与服务定义

### 消息 (msg/)

**Waypoint.msg** - 单个航点
- 基本信息: `name`, `id`, `map_name`
- GPS坐标: `lng`, `lat`
- 地图坐标: `x`, `y`, `yaw`
- 航点类型: `type`（1=普通导航点, 2=红绿灯, 3=充电桩, 4=地图切换点）
- 地图切换信息（type=4时）: `next_map_name`, `next_x`, `next_y`, `next_yaw`
- 附加参数: `tolerance`

**WaypointList.msg** - 航点列表
- `header` (std_msgs/Header)
- `waypoints` (Waypoint[])
- 任务元数据: `task_id`, `path`（路径名称，如path1/path2）, `total_waypoints`, `start_map_name`, `total_path`（路径总数）

**MapSwitchTrigger.msg** - 地图切换触发
- `current_map_name`, `next_map_name`
- `switch_pose` (geometry_msgs/PoseStamped)
- `current_waypoint_id`, `next_waypoint_id`

**RobotStatus.msg** - 机器人综合状态
- `header`, `vin`
- `task_status` (int32)
- GPS: `gps_lng`, `gps_lat`, `gps_alt`
- 车辆: `cur_speed`, `battery_capacity`

**Edge.msg** - 图的边
- `start_node_id` (int32): 起始节点ID
- `end_node_id` (int32): 结束节点ID
- `weight` (float32): 边权重/距离

**StartEndGraph.msg** - 图结构（MQTT下发给route_planner）
- `header` (std_msgs/Header)
- `start` (Waypoint): 起始点
- `end` (Waypoint): 目标点
- `nodes` (Waypoint[]): 图的所有节点
- `edges` (Edge[]): 图的所有边

### 服务 (srv/)

**StartProcess.srv** - 启动导航进程
- 请求: `process_name`, `map_name`
- 响应: `success` (bool), `message` (string)
- 支持进程: `re_localization`, `nav2_init_pose`, `liosam`, `navigation2`

**ShutdownProcess.srv** - 关闭导航进程
- 请求: `process_name`
- 响应: `success`, `message`

**GetProcessStatus.srv** - 查询进程状态
- 响应: `re_localization_running`, `liosam_running`, `nav2_init_pose_running`, `navigation2_running`, `message`

**PubNewPath.srv** - 请求发布备用路径（导航失败时由navigation_manager调用）
- 请求: `path_name`（当前路径名称，如path1）, `points` (Waypoint[]，已走过的航点)
- 响应: `success` (bool), `message` (string)

## 开发工作流

### 开发者A: 通信模块

**职责:**
1. 实现MQTT连接和消息处理
2. 解析航点JSON数据
3. 从ROS2主题聚合机器人状态
4. 实现心跳机制

**测试:**
```bash
# 使用模拟代理测试MQTT接收器
mosquitto_pub -h localhost -t robot/task -f test_task.json

# 监控状态输出
mosquitto_sub -h localhost -t robot/status
```

### 开发者B: 导航模块

**职责:**
1. 实现航点导航逻辑
2. 与Navigation2动作客户端集成
3. 实现地图切换协调
4. 管理进程生命周期

**测试:**
```bash
# 使用手动航点列表测试
ros2 topic pub /waypoint_list multi_map_navigation/WaypointList "{...}"

# 监控导航状态
ros2 topic echo /robot_state
```

## ROS2主题

### 发布的主题

- `/waypoint_list` (multi_map_navigation_msgs/WaypointList) - 航点任务列表（mqtt_task_receiver 或 route_planner 发布）
- `/robot_state` (std_msgs/String) - 机器人状态（idle/running）
- `/trigger_map_switch` (multi_map_navigation_msgs/MapSwitchTrigger) - 地图切换触发
- `/map_switch_complete` (std_msgs/Bool) - 地图切换完成状态

### 订阅的主题

- `/start_end_graph` (multi_map_navigation_msgs/StartEndGraph) - 图结构（route_planner 订阅）
- `/sensing/gnss/pose_with_covariance` - GPS数据
- `/battery_status` - 电池状态
- `/vehicle_status` - 车辆状态
- `/waypoint_list` (multi_map_navigation_msgs/WaypointList) - 航点任务列表（navigation_manager 订阅）
- `/map_switch_complete` (std_msgs/Bool) - 地图切换完成状态

## 故障排除

### MQTT连接问题

```bash
# 检查MQTT代理连接性
mosquitto_sub -h your_broker -t '#' -v

# 检查ROS2节点状态
ros2 node list
ros2 node info /mqtt_task_receiver
```

### 导航问题

```bash
# 检查Navigation2状态
ros2 topic echo /navigate_to_pose/_action/status

# 检查进程状态
ps aux | grep -E "relocalization|liosam|navigation2"
```

### 地图切换问题

```bash
# 监控地图切换事件
ros2 topic echo /trigger_map_switch
ros2 topic echo /map_switch_complete

# 检查进程管理器日志
ros2 run multi_map_navigation process_manager
```

## 测试

### 单元测试

```bash
# 运行测试
colcon test --packages-select multi_map_navigation

# 查看测试结果
colcon test-result --verbose
```

### 集成测试

1. 启动MQTT代理
2. 启动所有节点
3. 通过MQTT发送测试任务
4. 监控导航进度
5. 验证状态心跳

## 性能考虑

- **心跳频率**: 默认3秒，根据网络带宽调整
- **导航超时**: 默认每个航点300秒
- **地图切换超时**: 默认完整切换30秒
- **进程关闭**: 默认优雅关闭10秒

## 已知限制

1. 单机器人支持（多机器人需要扩展）
2. 地图文件必须预加载到机器人上
3. 地图切换期间无动态障碍物避让
4. 红绿灯检测（type=2）和充电桩对接（type=3）尚未实现
5. 进程启动顺序固定：re_localization → nav2_init_pose → liosam → navigation2
6. route_planner 尚未注册到 setup.py 的 entry_points 中

## 未来增强

- [ ] 多机器人协调
- [ ] 动态航点更新
- [ ] 地图预加载优化
- [ ] 导航失败的恢复行为
- [ ] 基于Web的监控仪表板
- [ ] 任务优先级和中断支持

## 贡献

### 代码风格

- Python代码遵循PEP 8
- 在适用的地方使用类型提示
- 为所有函数添加文档字符串
- 保持函数专注和模块化

### Git工作流

```bash
# 创建功能分支
git checkout -b feature/your-feature-name

# 进行更改并提交
git add .
git commit -m "更改描述"

# 推送并创建拉取请求
git push origin feature/your-feature-name
```

## 许可证

Apache-2.0

## 作者

- 开发者A: 通信模块
- 开发者B: 导航模块

## 支持

如有问题和疑问:
- 在项目仓库中创建issue
- 联系开发团队

## 致谢

- ROS2 Navigation2团队
- LIO-SAM开发者
- BehaviorTree.CPP社区
