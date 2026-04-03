# nav2_init_pose

`nav2_init_pose` 是 Car_jetson 项目中的初始位姿搜索功能包，作为 `re_localization` 的客户端使用。

它的职责不是直接做点云配准，而是先根据 GPS 粗定位结果，在地图坐标系中生成一批候选位置，然后不断调用 `/re_localization` 服务评估这些候选点的匹配分数，最终找到小车更接近真实位置的初始位姿，并持续发布 `map -> odom` 变换。

结合项目根目录说明，它位于整套导航链路中的重定位后处理环节：

`传感器 -> re_localization 服务 -> nav2_init_pose -> slam -> navigation2`

当前实现是一个 Python ROS2 节点，核心脚本为 `nav2_init_pose/nav2_init_pose.py`。

## 功能概览

- 订阅 GPS 数据，获得车辆当前位置的粗略估计
- 通过标定 CSV 将经纬度转换到地图坐标系
- 以 GPS 映射点为中心，在一定半径内进行候选位姿搜索
- 对每个候选点枚举多个朝向，并调用 `/re_localization` 获取匹配分数
- 先使用 PSO（粒子群优化）做全局搜索，再使用爬山法做局部精细优化
- 选择得分最优的结果作为车辆真实初始位姿
- 持续发布 `map -> odom` TF，供后续导航与定位模块使用

## 功能包结构

```text
nav2_init_pose/
├── launch/
│   ├── run.real_launch.py
│   └── run.sim_launch.py
├── nav2_init_pose/
│   ├── __init__.py
│   ├── nav2_init_pose.py
│   ├── nav2_init_pose_bck.py
│   ├── nav2_init_pose_copy.py
│   ├── visualize_score_map.py
│   ├── build_9.npz
│   └── log.txt
├── resource/
│   └── nav2_init_pose
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── record_gps_map2.csv
```

## 核心实现

- 主节点：`nav2_init_pose/nav2_init_pose.py`
- 实际环境启动文件：`launch/run.real_launch.py`
- 仿真启动文件：`launch/run.sim_launch.py`
- GPS 与地图坐标标定文件：`record_gps_map2.csv`

当前主流程如下：

1. 等待 `/re_localization` 服务可用
2. 订阅 GPS 话题，获取经纬度数据
3. 通过 `record_gps_map2.csv` 拟合 GPS 到地图坐标系的仿射变换
4. 将当前 GPS 点换算为地图坐标中的粗定位点
5. 以该点为中心，用 PSO 在设定半径内搜索若干候选位置
6. 对每个候选位置按固定角度步长枚举 yaw，并调用 `/re_localization`
7. 根据返回的 `fitness_score` 选择当前最优候选点
8. 在 PSO 结果基础上继续做局部爬山优化
9. 输出最终位姿，并持续广播 `map -> odom` TF

## 算法说明

### 1. GPS 到地图坐标转换

代码中的 `GpsToMapConverter` 会读取 CSV 标定点：

```text
纬度, 经度, map_x, map_y
```

先将经纬度转换为相对锚点的 ENU 平面坐标，再通过最小二乘法拟合 6 参数仿射变换，把 GPS 点映射到地图坐标系。

### 2. 候选点评估方式

对于任意候选位置 `(x, y)`，节点会按固定角度步长遍历多个 yaw：

- 构造 `ReLocalization` 请求
- 将该候选位姿作为初始位姿传给 `/re_localization`
- 获取服务返回的 `success` 与 `fitness_score`
- 选取分数最低的朝向作为该位置的评价结果

也就是说，`nav2_init_pose` 自身负责“找哪里值得试”，而 `re_localization` 负责“这个点配得准不准”。

### 3. 搜索策略

当前实现采用两阶段搜索：

- **PSO 全局搜索**：在 GPS 粗定位附近快速寻找较优候选区域
- **Hill Climb 精细搜索**：从 PSO 最优点出发，在局部继续细化

这样做的目的，是在避免直接陷入局部最优的同时，减少纯暴力遍历带来的服务调用量。

## 依赖

### ROS2 依赖

- rclpy
- geometry_msgs
- sensor_msgs
- tf2_ros
- `re_localization`

### Python 依赖

- numpy
- transforms3d

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select nav2_init_pose re_localization
source install/setup.bash
```

如果该节点需要直接调用 `/re_localization`，请确保 `re_localization` 已正确编译并能正常启动。

## 启动

### 1. 直接运行节点

该节点依赖 `map_csv` 参数，启动时需要显式传入：

```bash
ros2 run nav2_init_pose nav2_init_pose --ros-args -p map_csv:=/home/akun/workspace/Car_jetson/nav2/src/nav2_init_pose/record_gps_map2.csv
```

### 2. 使用 launch 启动

实际环境：

```bash
ros2 launch nav2_init_pose run.real_launch.py
```

仿真环境：

```bash
ros2 launch nav2_init_pose run.sim_launch.py
```

这两个 launch 文件会先包含 `re_localization` 的启动文件，再启动 `nav2_init_pose` 客户端节点。

## 话题与服务接口

### 订阅

- `/sensing/gnss/pose_with_covariance` (`sensor_msgs/msg/NavSatFix`)
  - 当前 GPS 输入话题
  - 节点会从中读取 `latitude` 和 `longitude`

### 调用服务

- `/re_localization` (`re_localization/srv/ReLocalization`)
  - 向服务端提交候选初始位姿
  - 获取对应的 `fitness_score` 和匹配后的位姿

### 发布

- `map -> odom` TF
  - 当搜索成功后，节点会周期性发布该变换
  - 发布周期为 0.1 秒

## 主要参数

以下参数定义在 `nav2_init_pose/nav2_init_pose.py` 中：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `map_csv` | `''` | GPS 与地图坐标标定文件路径，必填 |
| `fitness_score_threshold` | `0.05` | 理想匹配阈值，低于该值可提前停止搜索 |
| `pso_particles` | `30` | PSO 粒子数量 |
| `pso_iterations` | `6` | PSO 迭代轮数 |
| `pso_search_radius` | `10.0` | 以 GPS 初值为中心的搜索半径，单位米 |
| `pso_w` | `0.7` | PSO 惯性权重 |
| `pso_c1` | `1.5` | PSO 个体学习因子 |
| `pso_c2` | `1.5` | PSO 群体学习因子 |
| `pso_angle_step` | `30` | PSO 阶段朝向搜索步长，单位度 |
| `initial_step` | `1.0` | 爬山阶段初始平移步长，单位米 |
| `min_step` | `0.2` | 爬山阶段最小步长 |
| `step_reduction` | `0.5` | 爬山步长缩减因子 |
| `max_iterations` | `30` | 爬山阶段最大迭代次数 |
| `angle_step` | `15` | 爬山阶段角度步长 |
| `angle_fine_step` | `15` | 最终角度精细搜索步长 |

## 使用建议

### 1. 先保证 `re_localization` 服务稳定

`nav2_init_pose` 本质上是评分搜索客户端，自身不做点云匹配。
如果服务端地图未就绪、激光点云未到位或匹配参数不合适，客户端搜索效果会直接受影响。

### 2. 标定 CSV 质量决定初始搜索效果

`record_gps_map2.csv` 的标定精度会直接影响 GPS 到地图坐标的转换结果。
如果标定误差过大，PSO 搜索中心会偏离真实位置，导致搜索次数增加甚至失败。

### 3. 搜索半径与速度需要平衡

- 半径过小：GPS 偏差稍大时可能搜不到真实位置
- 半径过大：服务调用次数明显增加，初始化时间变长

通常应根据实际 GPS 误差范围设置 `pso_search_radius`。

### 4. `fitness_score_threshold` 越小越严格

当前实现中该阈值用于“提前结束搜索”的判断。
如果分数已经低于该阈值，说明已找到较理想解，节点会直接采用该结果。

## 已知注意事项

- `map_csv` 默认值为空字符串，若未传入，节点启动时会因找不到文件而失败
- `launch/run.real_launch.py` 与 `launch/run.sim_launch.py` 当前未显式传入 `map_csv` 参数，直接使用时通常需要补充参数配置
- `setup.py` 中注册了 `nav2_init_pose_room` 可执行入口，但当前包内未看到对应实现文件，直接调用可能失败
- 节点成功后只持续发布 TF，不会主动发布 `initialpose` 话题
- 当前搜索过程会频繁调用 `/re_localization`，启动前应确认服务端点云、地图与参数配置正确
- 代码中 GPS 话题名为 `/sensing/gnss/pose_with_covariance`，但消息类型使用的是 `NavSatFix`，使用前应确认上游发布类型与此一致

## 与项目的关系

根据项目根目录 `README.md`，`nav2_init_pose` 位于 `re_localization` 之后、SLAM 与 Navigation2 之前，负责把 GPS 粗定位与点云精匹配结合起来，自动寻找更可信的车辆初始位姿，为后续定位建图和导航流程提供稳定的 `map -> odom` 初始对齐结果。
