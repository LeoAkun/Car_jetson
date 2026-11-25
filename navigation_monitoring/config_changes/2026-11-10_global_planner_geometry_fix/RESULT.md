# 2026-11-10 全局规划、雷达几何与车辆模型修复结果

## 完成状态

四项修复均已写入实车工程并完成离线验证、构建和配置回归。本次没有启动传感器、ROS 节点、底盘、导航或录包回放。

## 1. 消除雷达二次反转

保留 Lakibeam 驱动的 `inverted=true`，将合并器配置改为：

- `flip1: false`
- `flip2: false`

修改文件：

- `/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml`

现有录包的纯离线重建结果：

- 原运行链 `flip=true` 与录包合并扫描 387/387 个有限角度格误差均小于 1 cm。
- 首败时旧几何的 PCA 主轴为 100.75°，Y/X 跨度比 5.09，表现为车前横墙。
- 改为 `flip=false` 后主轴为 165.36°，Y/X 跨度比 0.30，恢复为车辆左侧纵向边界。
- 另外四个失败时刻结论相同；五帧 `flip=false` 的 Y/X 跨度比为 0.105–0.200。

证据：

- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54/offline_flip_validation.md`
- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54/offline_flip_validation.json`
- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54/offline_flip_validation.png`

## 2. 下调全局膨胀并统一 footprint

全局膨胀修改：

- `inflation_radius: 2.0 → 0.55 m`
- `cost_scaling_factor: 1.5 → 5.0`

全局和局部代价地图现在使用同一轮廓：

```yaml
footprint: "[[-0.43, -0.28], [-0.43, 0.28], [0.43, 0.28], [0.43, -0.28]]"
footprint_padding: 0.02
```

含 padding 后的有效外框约为 0.90 m × 0.60 m，内切半径约 0.30 m、外接半径约 0.5408 m；0.55 m 膨胀略大于外接半径。原先 global 的 0.22 m 圆形半径和 local 的长宽轴互换矩形已删除。

这一轮廓依据 `base_link` 的 x 向前、y 向左约定及前后雷达 `x=±0.43 m、y=±0.28 m` 安装外参，是当前可审计的初值，尚未经过现场卷尺测量保险杠和车轮最外缘。

修改文件：

- `/home/akun/workspace/Car_jetson/nav2/src/nav2/config/nav2_real_params.yaml`

## 3. 统一 Smac、MPPI 和底盘转换器

统一软件约束：

- Smac `minimum_turning_radius: 1.0 m`
- MPPI `AckermannConstraints.min_turning_r: 1.0 m`
- 底盘转换器 `L: 0.5 m`
- 底盘转换器 `MAX_STEER_DEG: 26.565051177°`
- Smac `analytic_expansion_max_length: 5.0 m`

关系为：

`0.5 / tan(26.565051177°) = 1.0 m`

26.565°低于 JD01 CAN 层约 27°硬限，不再由配置请求 30°后交给 CAN 截断。0.8 km/h 最低非零速度补偿、2 km/h 普通导航上限均保持不变。

本轮录包反算的紧曲率样本约 0.8–1.0 m，1.0 m 是保守的软件一致初值。后轮转向的实际相位、比例和轮胎侧滑仍未独立标定，因此不能把 1.0 m 写成已实测的物理最小半径。

修改文件：

- `/home/akun/workspace/Car_jetson/real_ws/src/driver/car_control/cmdvel_2_ecu/cmdvel_2_ecu/convert.py`
- `/home/akun/workspace/Car_jetson/real_ws/src/driver/car_control/cmdvel_2_ecu/config/config.yaml`
- `/home/akun/workspace/Car_jetson/real_ws/src/driver/car_control/cmdvel_2_ecu/config/docking_config.yaml`

转弯反算：

- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54/turning_geometry_audit.json`

## 4. 构建和验证

构建成功：

- `ros2_laser_scan_merger`
- `cmdvel_2_ecu`
- `nav2`

回归结果：

- `test_docking_conversion.py`: 54 passed
- `test_navigation_recovery.py`: 46 passed，另有 2 条既有 NumPy `np.float` 弃用警告
- YAML 解析通过
- Python 编译检查通过
- `git diff --check` 通过
- Nav2 和合并器安装配置均解析到当前源码
- `cmdvel_2_ecu` 安装代码及两份安装配置与源码一致
- 雷达离线重建脚本重新执行成功

## 后续实车关注点

旧规划中部分离散路径段的估算曲率半径约 0.72–0.80 m，新的 1.0 m 约束会实际改变规划结果。下一轮 113→106→107 测试需要确认 Smac 能否在新约束下稳定生成路线，并复核车辆实际转向响应、全局代价地图及修正后的合并扫描。

按既有约定，明确开始实车测试后直接先录包并开启监测，不增加额外传感器预检测，不远程主动制动或切换驾驶模式。

## 快照与回滚

- 修改前快照：`before/`
- 修改后快照：`after/`
- 精确补丁：`change.patch`
- 哈希清单：`manifest.json`、`before_sha256.txt`、`after_sha256.txt`
- 回滚脚本：`rollback.sh`

