# multi_map_navigation/src

`multi_map_navigation/src` 是多地图导航子工程源码目录，包含：

- `multi_map_navigation`：主功能包，负责任务接收、路径规划、导航控制、地图切换、状态上报
- `multi_map_navigation_msgs`：自定义消息与服务定义包
- `map`：地图数据、地图网络拓扑以及各地图航点文件
- 若干测试脚本与说明文档

本文档聚焦 `src` 目录本身，方便快速了解源码结构、模块职责与使用方式。

---

## 1. 目录结构

```text
src/
├── README.md
├── CHANGELOG_API.md                 # 接口变更说明
├── PACKAGE_STRUCTURE.md             # 包结构说明
├── PROJECT_ARCHITECTURE.md          # 架构设计说明
├── QUICK_REFERENCE.md               # 快速参考
├── SUMMARY.md                       # 修改总结
├── kill_all.sh                      # 辅助关闭脚本
├── test_api_format.py               # API 格式测试脚本
├── test_map_switch.sh               # 地图切换测试脚本
├── test_nav2_server.py              # Nav2 服务测试脚本
├── test_pub_new_path.py             # 备用路径发布测试脚本
│
├── map/
│   ├── map_network.yaml             # 跨地图网络拓扑定义
│   ├── map1/
│   │   ├── map1_clean.pgm           # 栅格地图
│   │   ├── map1_clean.yaml          # 栅格地图元数据
│   │   ├── map1.csv                 # 地图航点/初始化数据
│   │   ├── map1_clean.pcd           # 点云地图
│   │   └── map1_raw.pcd             # 原始点云地图
│   └── map2/
│       ├── map2_clean.pgm
│       ├── map2_clean.yaml
│       ├── map2.csv
│       ├── map2_clean.pcd
│       └── map2_raw.pcd
│
├── multi_map_navigation/            # ament_python 功能包
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   ├── launch/
│   │   └── multi_map_navigation.launch.py
│   ├── config/
│   │   ├── mqtt_config.yaml
│   │   └── navigation_config.yaml
│   ├── multi_map_navigation/
│   │   ├── __init__.py
│   │   ├── mqtt_task_receiver.py
│   │   ├── route_planner.py
│   │   ├── navigation_manager.py
│   │   ├── map_switch_controller.py
│   │   ├── process_manager.py
│   │   ├── status_reporter.py
│   │   └── navigation_manager copy.py # 历史副本文件
│   ├── test/
│   │   ├── test_map_switch_controller.py
│   │   ├── test_map_switch_controller_real_deps.py
│   │   ├── test_mock_publisher.py
│   │   ├── test_mock_tf_publisher.py
│   │   ├── test_copyright.py
│   │   ├── test_flake8.py
│   │   ├── test_pep257.py
│   │   ├── test_mqtt_task_receiver/
│   │   ├── test_navigation_manager/
│   │   ├── test_process_manager/
│   │   ├── test_route_planner/
│   │   └── test_route_nav2_mapswitch_process/
│   ├── build/                       # colcon 构建产物
│   ├── install/                     # colcon 安装产物
│   └── log/                         # colcon 日志
│
└── multi_map_navigation_msgs/       # ament_cmake 消息/服务包
    ├── CMakeLists.txt
    ├── package.xml
    ├── README.md
    ├── msg/
    │   ├── Edge.msg
    │   ├── MapSwitchTrigger.msg
    │   ├── RobotStatus.msg
    │   ├── StartEndGraph.msg
    │   ├── Waypoint.msg
    │   └── WaypointList.msg
    └── srv/
        ├── GetProcessStatus.srv
        ├── PubNewPath.srv
        ├── ShutdownProcess.srv
        └── StartProcess.srv
```

> 说明：`multi_map_navigation/build/`、`install/`、`log/` 是构建产物目录，不属于手写源码，但当前也位于 `src` 子目录下。

---

## 2. 软件系统架构

从软件职责上看，当前系统可以划分为 5 个层次：接入层、任务与路径编排层、导航执行控制层、基础设施与接口层、数据层。

```text
┌─────────────────────────────────────────────────────────────┐
│                        接入层                               │
│  mqtt_task_receiver        status_reporter                 │
│  - 外部任务接入             - 外部状态上报                  │
└───────────────────────┬───────────────────────────────┬─────┘
                        │                               │
                        ▼                               ▲
┌─────────────────────────────────────────────────────────────┐
│                   任务与路径编排层                          │
│  route_planner                                              │
│  - 图建模                                                   │
│  - 主路径/备用路径规划                                      │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                   导航执行控制层                            │
│  navigation_manager                                         │
│  map_switch_controller                                      │
│  - 航点执行                                                 │
│  - 状态机控制                                               │
│  - 地图切换编排                                             │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                 基础设施与接口层                            │
│  process_manager                                            │
│  multi_map_navigation_msgs                                  │
│  launch/config                                              │
│  - 外部进程生命周期管理                                     │
│  - 消息与服务契约                                           │
│  - 启动与参数配置                                           │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                         数据层                              │
│                                    │
│  map1/, map2/                                               │
│  - 地图拓扑                                                 │
│  - 栅格地图 / 点云地图 / CSV 点位                           │
└─────────────────────────────────────────────────────────────┘
```

### 2.1 分层说明

#### 接入层

负责系统与外部平台的数据交换。

- `mqtt_task_receiver`
  - 从 MQTT 接入外部任务
  - 将外部任务转换为 ROS2 内部可消费的数据
- `status_reporter`
  - 订阅内部运行状态
  - 将机器人状态重新组织并上报到 MQTT

这一层的核心职责是“协议适配”和“边界数据转换”。

#### 任务与路径编排层

负责根据任务输入生成可执行导航路径。

- `route_planner`
  - 接收起点、终点、图结构
  - 构建拓扑图并生成路径
  - 在导航失败时生成备用路径

这一层解决的是“去哪里、怎么走”的问题。

#### 导航执行控制层

负责把规划结果转成实际的机器人行为。

- `navigation_manager`
  - 顺序执行航点
  - 管理任务状态、当前地图、当前航点索引
  - 在失败时请求新路径，在切图点触发地图切换
- `map_switch_controller`
  - 串联切图步骤
  - 控制旧地图导航栈下线与新地图导航栈上线

这一层是系统核心控制层，解决的是“何时执行、如何切换、异常后如何恢复”。

#### 基础设施与接口层

负责为上层提供稳定的运行支撑与统一接口契约。

- `process_manager`
  - 管理 `re_localization`、`nav2_init_pose`、`lio_sam`、`navigation2` 等外部进程
- `multi_map_navigation_msgs`
  - 统一定义 topic/service 的数据结构
- `launch/`、`config/`
  - 统一系统启动方式与参数配置

这一层为控制层和接入层提供“可调用能力”和“统一接口定义”。

#### 数据层

负责保存导航所依赖的静态或半静态数据。

- `map_network.yaml`：地图间拓扑关系
- `map1/`、`map2/`：地图、点云、CSV 点位数据

这一层提供规划与定位切换的基础数据支撑。

### 2.2 模块依赖关系

模块依赖关系可概括为：

- 接入层依赖接口层定义的数据结构
- 编排层依赖数据层中的地图与拓扑数据
- 控制层依赖编排层输出的路径结果
- 控制层通过基础设施层驱动外部导航相关进程
- 接入层读取控制层和传感器侧状态，对外上报

即：

```text
外部系统 <-> 接入层 <-> 编排/控制层 <-> 基础设施层 <-> 地图与配置数据
```

### 2.3 分层设计价值

采用上述分层后，系统具备以下特点：

- **边界清晰**：MQTT 接入、路径规划、导航执行、进程管理职责分离
- **易于扩展**：后续若替换通信协议或路径规划算法，影响范围更可控
- **易于测试**：可分别对规划层、控制层、进程层做独立测试
- **利于维护**：问题定位时可快速判断属于接入、规划、控制还是基础设施问题

---

## 3. 功能包说明

### 2.1 `multi_map_navigation`

主业务包，负责多地图导航主流程。

#### 核心节点

- `mqtt_task_receiver.py`
  - 接收 MQTT 任务
  - 解析任务图/航点相关数据
  - 向 ROS2 发布任务输入消息

- `route_planner.py`
  - 订阅 `/start_end_graph`
  - 基于 `networkx` 构建图并计算路径
  - 发布 `/waypoint_list`
  - 提供 `/route_planner/pub_new_path` 服务，用于导航失败后的备用路径切换

- `navigation_manager.py`
  - 订阅 `/waypoint_list`
  - 按序驱动 Nav2 执行导航
  - 维护任务状态、当前航点、当前地图
  - 在地图切换点触发 `/trigger_map_switch`
  - 通过服务调用 `process_manager` 和 `route_planner`

- `map_switch_controller.py`
  - 订阅 `/trigger_map_switch`
  - 执行地图切换流程：关闭旧导航栈 → 等待清理 → 启动新导航栈 → 校验状态
  - 发布 `/map_switch_complete`

- `process_manager.py`
  - 提供进程管理服务
  - 管理 `re_localization`、`nav2_init_pose`、`lio_sam`、`navigation2` 等外部进程生命周期

- `status_reporter.py`
  - 汇总机器人运行状态、GPS、电池、车速等数据
  - 发布 `/robot_status`
  - 向 MQTT 上报车辆状态心跳

#### 启动文件

- `launch/multi_map_navigation.launch.py`
  - 启动以下节点：
    - `process_manager`
    - `mqtt_task_receiver`
    - `status_reporter`
    - `route_planner`
    - `navigation_manager`
    - `map_switch_controller`

#### 配置文件

- `config/mqtt_config.yaml`
  - MQTT 连接参数
  - 任务主题、状态主题
  - VIN、上报周期、状态源话题等

- `config/navigation_config.yaml`
  - 导航容差、超时、重试参数
  - 地图目录
  - 进程启动参数
  - 共享话题名称

---

### 2.2 `multi_map_navigation_msgs`

定义工程中使用的 ROS2 自定义消息与服务。

#### 消息

- `Waypoint.msg`：单个航点定义
- `WaypointList.msg`：一组待执行航点及任务元数据
- `MapSwitchTrigger.msg`：地图切换触发消息
- `RobotStatus.msg`：机器人状态汇总
- `Edge.msg`：图边定义
- `StartEndGraph.msg`：起点、终点与路径图结构

#### 服务

- `StartProcess.srv`：启动指定进程
- `ShutdownProcess.srv`：关闭指定进程
- `GetProcessStatus.srv`：查询进程状态
- `PubNewPath.srv`：请求发布一条新路径

---

## 4. 地图数据目录说明

`map/` 用于存放多地图导航依赖的数据文件。

### 3.1 `map_network.yaml`

定义跨地图的网络拓扑关系，供路径规划逻辑使用。

### 3.2 `map1/`、`map2/`

每张地图通常包含：

- `*_clean.pgm`：2D 栅格地图
- `*_clean.yaml`：栅格地图元信息
- `*.csv`：地图相关点位/初始化位姿数据
- `*_clean.pcd`：处理后的点云地图
- `*_raw.pcd`：原始点云地图

---

## 5. 系统主流程

整体数据流如下：

```text
MQTT任务
  ↓
mqtt_task_receiver
  ↓
/start_end_graph 或 /waypoint_list
  ↓
route_planner
  ↓
/waypoint_list
  ↓
navigation_manager
  ├─ 调用 Nav2 导航
  ├─ 调用 /route_planner/pub_new_path 获取备用路径
  └─ 发布 /trigger_map_switch
         ↓
   map_switch_controller
         ↓
   process_manager
         ↓
   re_localization / nav2_init_pose / lio_sam / navigation2

status_reporter
  ↑
机器人状态 / GPS / 电池 / 车速
  ↓
MQTT状态上报
```

---

## 6. 常用话题与服务

### 5.1 主要话题

- `/start_end_graph`：路径规划输入
- `/waypoint_list`：导航航点序列
- `/robot_state`：机器人运行状态
- `/robot_status`：机器人综合状态
- `/trigger_map_switch`：触发地图切换
- `/map_switch_complete`：地图切换完成通知
- `/sensing/gnss/pose_with_covariance`：GNSS 数据
- `/battery_status`：电池状态
- `/vehicle_status`：车速/车辆状态

### 5.2 主要服务

- `/process_manager/start_process`
- `/process_manager/shutdown_process`
- `/process_manager/get_status`
- `/route_planner/pub_new_path`

---

## 7. 构建方式

建议在工作区根目录执行：

```bash
cd /home/akun/workspace/Car_jetson
colcon build --packages-select multi_map_navigation_msgs multi_map_navigation
source install/setup.bash
```

如果仅构建当前子工程，也可在 `multi_map_navigation/` 根目录执行 `colcon build`。

---

## 8. 启动方式

### 7.1 启动整套系统

```bash
ros2 launch multi_map_navigation multi_map_navigation.launch.py
```

### 7.2 单独启动节点

```bash
ros2 run multi_map_navigation process_manager
ros2 run multi_map_navigation mqtt_task_receiver
ros2 run multi_map_navigation status_reporter
ros2 run multi_map_navigation route_planner
ros2 run multi_map_navigation navigation_manager
ros2 run multi_map_navigation map_switch_controller
```

---

## 9. 测试与辅助文件

`src` 根目录和 `multi_map_navigation/test/` 下包含若干测试与模拟脚本，用于：

- 校验消息/API 格式
- 测试地图切换流程
- 模拟 Nav2、TF、路径发布、进程管理等依赖
- 执行 lint 与基础单元测试

其中：

- `test_map_switch.sh`：地图切换联调脚本
- `test_api_format.py`：接口格式校验
- `test_nav2_server.py`：Nav2 服务侧验证
- `test_pub_new_path.py`：备用路径服务测试

---

## 10. 相关文档

如需继续深入，可结合以下文档阅读：

- `PROJECT_ARCHITECTURE.md`：整体架构说明
- `PACKAGE_STRUCTURE.md`：包拆分说明
- `QUICK_REFERENCE.md`：快速查阅
- `CHANGELOG_API.md`：接口变更记录
- `multi_map_navigation_msgs/README.md`：状态消息说明

---

## 11. 备注

- `multi_map_navigation/multi_map_navigation/navigation_manager copy.py` 为历史副本文件，通常不作为正式入口。
- 当前 `src` 下包含构建产物目录，阅读源码时建议优先关注：
  - `multi_map_navigation/multi_map_navigation/`
  - `multi_map_navigation/launch/`
  - `multi_map_navigation/config/`
  - `multi_map_navigation_msgs/`
  - `map/`
