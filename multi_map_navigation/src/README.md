# 多地图导航系统

基于ROS2的多地图导航系统，具有MQTT通信和动态地图切换功能。

## 概述

该系统使机器人能够无缝地在多个地图中导航，通过MQTT接收航点任务并实时报告状态。它处理自动地图切换、进程管理和导航协调。

## 功能特性

- **MQTT通信**: 通过MQTT接收导航任务并发布机器人状态
- **多地图导航**: 在不同地图的航点之间导航
- **动态地图切换**: 在导航过程中自动切换地图
- **进程管理**: 管理导航堆栈组件的生命周期
- **实时状态报告**: 基于心跳的状态更新（GPS、电池、速度、状态）
- **模块化架构**: 通信和导航模块之间的清晰分离

## 架构

### 模块概览

```
┌─────────────────────────────────────────────────────────────┐
│                    MQTT代理                                 │
└─────────────────┬───────────────────────────┬───────────────┘
                  │                                         │
         (航点任务)                                      (状态心跳)
                  │                                         │
                  ▼                                         ▲
┌─────────────────────────────┐                    ┌──────────────────────────┐
│  通信模块 (A)               │                     │  通信模块 (A)            │
│  - mqtt_task_receiver       │                    │  - status_reporter       │
└─────────────┬───────────────┘                    └──────────▲───────────────┘
              │                                        │                    │
              │ /waypoint_list                         │ /robot_state       │
              │                                        │                    │/sensing/gnss/pose_with_covariance
              ▼                                        │                    │/battery_status
┌─────────────────────────────────────────────────────────────┐             │/vehicle_status
│           导航模块 (B)                                      │              │
│  ┌──────────────────────────────────────────────────────┐  │              │
│  │  navigation_manager                                   │  │             │
│  │  - 航点排序                                          │  │               │
│  │  - Nav2目标发送                                      │  │               │
│  │  - 地图切换检测                                      │  │               │
│  └────────────┬─────────────────────────────────────────┘  │              │
│               │ /trigger_map_switch                         │             │
│               ▼                                             │             │
│  ┌──────────────────────────────────────────────────────┐  │              │
│  │  map_switch_controller                                │  │             │
│  │  - 协调地图切换                                      │  │               │
│  │  - 进程关闭/启动                                     │  │               │
│  └────────────┬─────────────────────────────────────────┘  │              │ 
│               │                                             │             │
│               ▼                                             │             │
│  ┌──────────────────────────────────────────────────────┐  │              │
│  │  process_manager                                      │  │             │
│  │  - 启动/关闭 re_localization                         │  │               │
│  │  - 启动/关闭 lio_sam                                 │  │               │
│  │  - 启动/关闭 navigation2                             │  │               │
│  └──────────────────────────────────────────────────────┘  │              │
└─────────────────────────────────────────────────────────────┘             │
```

## 项目结构

```
multi_map_navigation/
├── src/
│   ├── communication/              # 开发者A
│   │   ├── mqtt_task_receiver.py   # 从MQTT接收任务
│   │   └── status_reporter.py      # 向MQTT报告状态
│   ├── navigation/                 # 开发者B
│   │   ├── navigation_manager.py   # 管理航点导航
│   │   ├── map_switch_controller.py # 处理地图切换
│   │   └── process_manager.py      # 管理ROS2进程
│   └── main.py                     # 主入口
├── msg/
│   ├── Waypoint.msg                # 单个航点定义
│   ├── WaypointList.msg            # 航点列表
│   └── MapSwitchTrigger.msg        # 地图切换触发
├── config/
│   ├── mqtt_config.yaml            # MQTT配置
│   └── navigation_config.yaml      # 导航配置
├── launch/
│   └── multi_map_navigation.launch.py
├── CMakeLists.txt
├── package.xml
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
colcon build --packages-select multi_map_navigation
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
# 通信模块（开发者A）
ros2 run multi_map_navigation mqtt_task_receiver.py
ros2 run multi_map_navigation status_reporter.py

# 导航模块（开发者B）
ros2 run multi_map_navigation navigation_manager.py
ros2 run multi_map_navigation map_switch_controller.py
ros2 run multi_map_navigation process_manager.py
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
    "type": 0
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
    "type": 1
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
    "type": 0
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
      "type": 0
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
      "type": 1
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
- `type`: 航点类型（0=正常导航点, 1=地图切换点）
- `next_map_name`: 下一张地图名称（仅type=1时需要）
- `next_x`: 当前点在下一张地图坐标系下的X坐标（仅type=1时需要）
- `next_y`: 当前点在下一张地图坐标系下的Y坐标（仅type=1时需要）
- `next_yaw`: 当前点在下一张地图坐标系下的偏航角（仅type=1时需要）

### 状态消息（发送）

主题: `robot/status`

```json
{
  "gps": {
    "latitude": 34.052235,
    "longitude": -118.243683,
    "altitude": 100.0
  },
  "battery": {
    "percentage": 85.5,
    "voltage": 24.5,
    "current": 2.3
  },
  "velocity": {
    "linear": 0.5,
    "angular": 0.1
  },
  "state": "running",
  "timestamp": 1706428800
}
```

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

- `/waypoint_list` (multi_map_navigation/WaypointList) - 航点任务列表
- `/robot_state` (std_msgs/String) - 机器人状态（idle/running）
- `/trigger_map_switch` (multi_map_navigation/MapSwitchTrigger) - 地图切换触发
- `/map_switch_complete` (std_msgs/Bool) - 地图切换完成状态

### 订阅的主题

- `/gps/fix` (sensor_msgs/NavSatFix) - GPS数据
- `/battery_state` (sensor_msgs/BatteryState) - 电池状态
- `/cmd_vel` (geometry_msgs/Twist) - 速度命令
- `/goal_reached` (来自Nav2动作) - 导航目标状态

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
ros2 run multi_map_navigation process_manager.py
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

1. 仅支持顺序航点导航（无并行路径）
2. 单机器人支持（多机器人需要扩展）
3. 地图文件必须预加载到机器人上
4. 地图切换期间无动态障碍物避让

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
