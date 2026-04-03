# re_localization

`re_localization` 是 Car_jetson 项目中的激光点云重定位功能包，用于在已有全局点云地图上，根据当前激光雷达点云和给定初始位姿执行一次点云配准，返回更准确的定位结果。

结合项目根目录说明，它位于整套导航链路的前段：

`传感器 -> re_localization 服务 -> nav2_init_pose -> slam -> navigation2`

当前实现基于 PCL/ndt_omp_ros2，核心节点为生命周期节点 `pcl_localization`，对外提供 `re_localization` 服务。

## 功能概览

- 基于点云地图进行重定位
- 支持 `NDT`、`NDT_OMP`、`GICP`、`GICP_OMP` 配准方法
- 支持从 PCD 文件加载地图，或订阅 `/map` 点云地图
- 支持通过服务传入初始位姿并返回匹配结果
- 可选使用里程计预测位姿
- 可选使用 IMU 做点云去畸变
- 首次调用时缓存一帧激光点云，后续服务调用可复用缓存点云

## 功能包结构

```text
re_localization/
├── include/pcl_localization/
│   ├── pcl_localization_component.hpp
│   └── lidar_undistortion.hpp
├── launch/
│   ├── run.real_launch.py
│   └── run.sim_launch.py
├── param/
│   ├── localization_real.yaml
│   └── localization_sim.yaml
├── src/
│   ├── pcl_localization_component.cpp
│   └── pcl_localization_node.cpp
├── srv/
│   └── ReLocalization.srv
└── rviz/
    └── localization.rviz
```

## 核心实现

- 节点入口：`src/pcl_localization_node.cpp`
- 生命周期节点实现：`src/pcl_localization_component.cpp`
- 服务定义：`srv/ReLocalization.srv`
- 实际环境启动文件：`launch/run.real_launch.py`
- 实际环境参数：`param/localization_real.yaml`

服务回调中会执行以下流程：

1. 接收外部传入的初始位姿
2. 检查地图是否就绪
3. 等待一帧激光点云（首次调用）并做体素下采样、距离裁剪
4. 以初始位姿作为配准初值进行点云匹配
5. 计算匹配结果与 fitness score
6. 返回重定位后的位姿

## 依赖

### ROS2 依赖

- rclcpp
- rclcpp_lifecycle
- lifecycle_msgs
- geometry_msgs
- sensor_msgs
- nav_msgs
- tf2_ros
- tf2_sensor_msgs
- tf2_geometry_msgs
- tf2_eigen
- pcl_conversions
- std_msgs

### 第三方依赖

- PCL
- OpenMP
- yaml-cpp
- `ndt_omp_ros2`

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select re_localization
source install/setup.bash
```

如果 `ndt_omp_ros2` 尚未编译，请先确保该依赖已在同一工作空间中可用。

## 启动

### 1. 实际环境启动

```bash
ros2 launch re_localization run.real_launch.py
```

也可以在启动时覆盖地图路径：

```bash
ros2 launch re_localization run.real_launch.py map_path:=/absolute/path/to/map.pcd
# 示例：ros2 launch re_localization run.real_launch.py  map_path:=/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/map2/map2_raw.pcd
```

`run.real_launch.py` 会自动驱动生命周期节点完成 `configure -> activate`。

### 2. 仿真启动

```bash
ros2 launch re_localization run.sim_launch.py
```

注意：当前 `launch/run.sim_launch.py` 中的包名仍是 `pcl_localization_ros2`，如果直接用于当前工程，通常需要先按本包名 `re_localization` 做适配。

## 服务接口

服务名：`/re_localization`

服务定义：

```srv
# Request
geometry_msgs/PoseWithCovarianceStamped initial_pose
---
# Response
bool success
float64 fitness_score
geometry_msgs/PoseWithCovarianceStamped pose
```

### 调用说明

- 请求中的 `initial_pose.header.frame_id` 必须与参数 `global_frame_id` 一致，默认是 `map`
- 节点需要先拿到地图后才能正常执行匹配
- 首次服务调用会等待点云，超时时间为 3 秒
- 如果 3 秒内没有接收到激光点云，返回 `success=false`

### 示例

```bash
ros2 service call /re_localization re_localization/srv/ReLocalization "{
  initial_pose: {
    header: {frame_id: map},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }
  }
}"
```

## 话题接口

### 订阅

- `/map` (`sensor_msgs/msg/PointCloud2`)
  - 当 `use_pcd_map=false` 时，从话题接收地图
- `/odom` (`nav_msgs/msg/Odometry`)
  - 当 `use_odom=true` 时用于位姿预测
- `/imu_raw_simtime2realtime` (`sensor_msgs/msg/Imu`)
  - 当 `use_imu=true` 时用于点云去畸变
- 激光点云话题由参数 `lidar_topic` 指定
  - 实际环境默认：`/rslidar_Pointcloud2`
  - 仿真默认：`/livox/pointcloud`

### 发布

- `/initial_map` (`sensor_msgs/msg/PointCloud2`)
  - 当 `use_pcd_map=true` 时发布初始地图
- `/path` (`nav_msgs/msg/Path`)
  - 已创建发布器，但当前代码未持续写入路径点

## 主要参数

以下为 `param/localization_real.yaml` 中的主要参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `registration_method` | `NDT_OMP` | 配准方法，可选 `NDT` / `NDT_OMP` / `GICP` / `GICP_OMP` |
| `score_threshold` | `2.0` | 匹配质量阈值，越小要求越严格 |
| `ndt_resolution` | `1.5` | NDT 体素分辨率 |
| `ndt_step_size` | `0.1` | NDT 迭代步长 |
| `ndt_num_threads` | `8` | `NDT_OMP` 使用线程数 |
| `transform_epsilon` | `0.01` | 配准收敛阈值 |
| `voxel_leaf_size` | `0.3` | 输入点云体素下采样尺寸 |
| `scan_max_range` | `60.0` | 点云最大有效距离 |
| `scan_min_range` | `0.5` | 点云最小有效距离 |
| `use_pcd_map` | `true` | 是否直接加载 PCD 地图 |
| `map_path` | `/home/akun/workspace/Car_jetson/GlobalMap3.pcd` | 地图路径 |
| `use_odom` | `false` | 是否使用里程计辅助 |
| `use_imu` | `false` | 是否使用 IMU 去畸变 |
| `enable_debug` | `true` | 是否输出调试信息 |
| `global_frame_id` | `map` | 全局坐标系 |
| `odom_frame_id` | `odom` | 里程计坐标系 |
| `base_frame_id` | `base_link` | 机器人本体坐标系 |
| `lidar_topic` | `/rslidar_Pointcloud2` | 激光点云话题 |

## 使用建议

### 1. 关于初始位姿

该功能包不是全局定位搜索，而是“给定一个近似初值后进行局部精配准”。
初始位姿越接近真实位置，重定位成功率越高。

### 2. 关于地图来源

- 若使用固定 PCD 地图，推荐设置 `use_pcd_map=true`
- 若已有上游节点发布点云地图，可设置 `use_pcd_map=false` 并订阅 `/map`

### 3. 关于配准方法选择

- `NDT_OMP`：通常适合当前项目，速度和效果较平衡
- `GICP/GICP_OMP`：对局部精细配准可能更好，但计算开销通常更高

### 4. 关于 score_threshold

当前实现中 `fitness_score` 实际由内点率换算得到：

- 内点率越高，`fitness_score` 越小
- `fitness_score > score_threshold` 时会报警告
- 即使超过阈值，当前代码仍会返回结果，调用方需要自行决定是否采用

## 已知注意事项

- `run.sim_launch.py` 仍保留旧包名，直接使用前建议先检查
- `path` 发布器已创建，但当前实现未持续维护轨迹
- `set_initial_pose` 相关参数已声明，但当前主流程以服务传入位姿为主
- 首次调用后会缓存一帧点云，若环境变化较大，可能需要重启节点以获取新的首帧点云

## 与项目的关系

根据项目根目录 `README.md`，`re_localization` 属于 `utils/` 下的重要工具模块，主要承担导航系统启动前的重定位职责，为后续 `nav2_init_pose`、SLAM 和 Navigation2 提供更可靠的初始位姿输入。
