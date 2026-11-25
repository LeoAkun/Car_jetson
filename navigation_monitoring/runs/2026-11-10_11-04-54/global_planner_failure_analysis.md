# 2026-11-10 全局规划反复失败分析

> 修复已于 2026-11-10 完成并构建验证。详见
> `/home/akun/workspace/Car_jetson/navigation_monitoring/config_changes/2026-11-10_global_planner_geometry_fix/RESULT.md`。

## 结论

本轮 106→107 段全局规划反复失败的首要原因已经锁定：

1. 前、后 Lakibeam 驱动以 `inverted=true` 发布扫描，驱动内部已经把束序反转一次。
2. `ros2_laser_scan_merger` 又配置 `flip1=true`、`flip2=true`，合并时再次按倒序读取 ranges。
3. 第二次反转把车辆左侧的纵向路侧边界错误投影成车前横向障碍。
4. 全局障碍层订阅 `/laser/scan_merge_laser`，将这批错误点标为致命障碍；2.0 m 全局膨胀继续扩大软代价区。
5. Smac Hybrid-A* 在这一局部几何下反复耗尽 1,000,000 次迭代，最终 107 任务失败。

本分析仅使用现有日志、录包、源码和配置；没有启动传感器、ROS、底盘、导航或回放，也没有修改运行参数。

## 直接日志证据

- 107 目标：`(122.66, -249.50)`。
- 11:13:07.809 出现第一次规划失败。
- 107 段共出现 27 次：`GridBased: failed to create plan, exceeded maximum iterations.`
- `GridBased` 是插件别名，实际插件为 `nav2_smac_planner/SmacPlannerHybrid`。
- 清全局代价地图另有 6 次约 30 ms 服务超时；这些发生在规划失败之后，是恢复链放大因素，不是第一次失败的触发源。

日志：

- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54/nav2_2026-11-10_11-06-34.log`

## 首败时的全局代价地图

11:13:07.809 附近：

- 静态通道宽度约 3.8 m。
- 动态更新新增 449 个代价为 253/254 的栅格，组成约 3.7 m × 2.2 m 的横向高代价簇。
- `y=-182.5` 截面只剩约 0.2 m 和 0.4 m 的缝隙，小于当前全局规划器使用的 0.44 m 机器人直径。
- 起点约 `(117.00, -180.48)`，中心代价 37；目标代价 26。两者均不在致命区或未知区。
- 后续失败时起点代价约 53–112，目标代价仍约 26。
- 若把机器人当作无体积点，0–252 栅格仍有二维连通；加入机器人尺寸和 Hybrid 运动学约束后，局部横墙使通道不可穿越并迅速扩大搜索。

因此，问题不是目标点本身被占用，也不是静态地图永久断路。

## 雷达几何证据

首败时 2.5 m 范围内：

- 原始前雷达约 450 点，原始后雷达约 3 点。
- 合并扫描有 177 个近距离输出角度格。
- 按合并器源码和实车参数逐格离线重建，172/177 来自前雷达，5/177 来自后雷达；前雷达占 97.2%。
- 231 个共同有限输出格与录包值误差小于 1 cm，来源归因可靠。

几何变化：

- 按原始前雷达消息的角度语义和外参变换后，近点位于车辆左侧，PCA 主轴约 167.2°，表现为沿车身方向的纵向边界。
- 实际合并扫描中的同一批近点位于车辆前方，PCA 主轴约 100.7°，变成横向“墙”。
- 首败前后的成功帧、第二组失败帧也重复出现纵向边界被变成横向障碍的现象。

录包能证明几何被错误旋转，但不能仅凭点云给现场物体命名；它可能是路沿或其他路侧结构。

## 二次反转证据

实车日志明确显示两台驱动均使用 `inverted:True`：

- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54/real_drivers.log:63`
- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54/real_drivers.log:76`

驱动在 `inverted=true` 时写入倒序索引：

- `/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/Lakibeam_ROS2_Driver-main/src/lakibeam1_scan.cpp:247`
- `/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/Lakibeam_ROS2_Driver-main/src/lakibeam1_scan.cpp:259`

合并器配置又启用了两路 flip：

- `/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml:3`
- `/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml:4`

合并器在 flip 为 true 时再次倒序取值：

- `/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/ros2_laser_scan_merger/src/main.cpp:93`
- `/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/ros2_laser_scan_merger/src/main.cpp:175`

当前实车 `run.launch.py` 确实启动这套驱动和合并器，安装配置也是源码配置的软链接且哈希一致，因此这不是旧参考工程误判。

## 2.0 m 膨胀半径的作用

当前全局配置：

- `robot_radius: 0.22`
- `inflation_radius: 2.0`
- `cost_scaling_factor: 1.5`
- 障碍层订阅 `/laser/scan_merge_laser`
- `obstacle_max_range: 2.5`

106→107 路段中心线到静态占据边界的净空约 1.51–1.99 m，所以中心线整段都处于 2.0 m 膨胀代价场内。2.0 m 膨胀不会凭空生成首败横墙的 254 致命障碍，但会把错误横墙周围大范围铺成 1–252 的软代价，增加 Hybrid 搜索成本和对起姿的敏感性。

结论：当前 2.0 m 全局膨胀半径不适合这条道路尺度，需要下调；最终数值应与修正后的整车 footprint 一起确定。

参数文件：

- `/home/akun/workspace/Car_jetson/nav2/src/nav2/config/nav2_real_params.yaml:446`
- `/home/akun/workspace/Car_jetson/nav2/src/nav2/config/nav2_real_params.yaml:526`

## 其他放大因素

- 全局地图约 4097 × 6354、分辨率 0.1 m；64 个航向离散对应约 16.66 亿个理论状态。100 万次迭代只覆盖约 0.060%。
- 全局规划器和 MPPI 的最小转弯半径为 0.40 m，而当前 0.5 m 轴距及约 27°–30°最大转角对应约 0.87–0.98 m，规划模型比底盘实际灵活。
- 全局使用 0.22 m 圆形半径，局部使用约 `x=±0.27, y=±0.43` 的多边形，规划和控制的车辆几何不一致。
- 全局障碍层量程为 2.5 m，局部为 7 m。
- 全局层只有 observation source 级 `min_obstacle_height=-5.0`，层级参数仍可能使用默认 0，可能过滤 map 坐标下的负高度扫描点。
- `cache_obstacle_heuristic=true` 与动态障碍层反复变化的场景不理想。

这些因素会增加失败概率，但本轮反复失败的首要、可复现原因是合并扫描的二次反转。

## 建议修复顺序

1. 保留驱动 `inverted=true`，把合并器 `flip1`、`flip2` 改为 `false`，使整条链只反转一次。原始前后扫描还可能被其他模块使用，因此优先取消合并器中的重复反转。
2. 用现有录包离线验证修正后的合并几何：左侧纵向边界应保持纵向，首败位置的车前横墙应消失。
3. 将全局膨胀半径从 2.0 m 下调，并同步核准整车 footprint；不要只靠提高最大迭代数掩盖错误障碍。
4. 统一 Smac、MPPI 和底盘的最小转弯半径，再处理全局/局部障碍范围、高度过滤和启发式缓存。
5. 延长清全局代价地图服务超时，避免规划失败后恢复动作继续超时。

