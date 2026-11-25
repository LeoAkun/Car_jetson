# 2026-11-16 下午最后一轮倒车失败复核

## 结论

最后一项实际导航任务是 `task_1794818067`，路线为 map2 的 `53 → 54 → 58 → 57`。车辆已自主完成 53 和 54；前往第三个航点 58 时，控制器反复无法生成安全轨迹，全局规划器又持续判定当前起点处于致命代价格。行为树因此执行了配置中的 `BackUp` 恢复动作，现场看到的倒车属于导航恢复。倒车路径随即被本地代价地图的碰撞检查判为有碰撞风险，`BackUp` 失败；清理全局代价地图又多次超时，车辆没有脱离致命区，最终目标和自动备用路线均以状态 6（ABORTED）结束。

这不是启动重定位失败：`map → odom` 已建立，Nav2 已就绪，并且前两个航点成功到达。

## 时间线

| 时间 | 事件 |
|---|---|
| 16:34:27 | 收到四航点任务 `53 → 54 → 58 → 57` |
| 16:34:52 | `map → odom` 就绪 |
| 16:35:50 | 航点 53 成功 |
| 16:39:51 | 航点 54 成功，任务完成一半 |
| 16:40:03 | 航点 58 被 Nav2 接受 |
| 16:40:14 起 | 第三段出现 MPPI `Optimizer fail to compute path` |
| 16:40:16 起 | 第三段反复出现 `Starting point in lethal space` |
| 16:41:09 | 行为树的 `BackUp` 恢复被碰撞检查终止：`Collision Ahead`、`backup failed` |
| 16:41:44 | 航点 58 失败，状态 6；管理器自动请求备用路线 |
| 16:41:50 起 | 备用路线的第一个目标 54 从当前位姿规划，仍持续报告起点在致命区 |
| 16:42:10～16:42:29 | 备用路线四次 `BackUp` 均因碰撞检查失败 |
| 16:42:35 | 备用路线状态 6，任务结束并关闭导航进程 |

## 倒车为什么会出现

实车启动文件明确加载 `general_nav_to_pose.xml`。该行为树在常规规划、跟踪、清图和等待都不能恢复时，会执行 `BackUp backup_dist="0.30" backup_speed="0.24"`，即计划后退 0.30 m、请求速度 0.24 m/s。

规划与控制参数本身也允许负速度：MPPI 的 `vx_min` 为 `-0.555555... m/s`，Smac 使用可生成倒车段的 `REEDS_SHEPP`，同时 `PreferForwardCritic` 被关闭。由于日志明确记录了 `backup failed`，本轮末段至少可以确认行为树调用了倒车恢复；但现有录包没有 `/cmd_vel` 和底盘反馈，无法把现场观察到的全部实际倒车距离逐帧归因给行为树或 MPPI 路径。

## 为什么倒车后仍失败

1. 原始故障没有消失：本轮共记录 37 次 `Starting point in lethal space`，第三航点和备用路线都无法从当前位姿生成可行全局路径。
2. 倒车恢复自身被本地碰撞检查终止：共 5 次 `Collision Ahead` 和 5 次 `backup failed`。这说明倒车方向的预测足迹与本地代价地图障碍相交；日志不能区分障碍是人员、真实路沿、近车体回波还是残留格。
3. 全局清图恢复共超时 13 次。当前 `default_server_timeout: 30` 在 Nav2 源码中按毫秒解释，即只有 30 ms，现场负载下过短，致命状态没有通过恢复流程稳定清除。
4. 第一次失败后，管理器自动下发了从 54 开始的备用路线；当前位姿仍在致命区，所以备用路线也无法启动有效行驶。

## 能确定与不能确定的部分

能确定的直接原因是：**第三段规划/控制不可行 → 行为树倒车恢复 → 倒车碰撞检查失败 → 起点致命状态持续 → 原路线与备用路线均中止。**

不能用本轮数据唯一确定最初的致命格来自定位跳变、静态地图边缘还是全局障碍层。16:34 的 rosbag 因 ROS 网络隔离只录到 12 条参数/日志消息，`/cmd_vel`、TF、里程计、激光和代价地图均为零条；LIO 节点日志也没有输出足以复算位姿连续性的数值。因此不能把本轮进一步定性为“重定位失败”或“再次发生位置跳变”。

## 证据文件

- 导航管理器：`/home/akun/.ros/log/python3_22578_1794816331692.log`
- Nav2：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-38-38_manual_publish_monitor/nav2_2026-11-16_16-34-54.log`
- 行为树：`/home/akun/workspace/Car_jetson/nav2/src/nav2/config/general_nav_to_pose.xml`
- Nav2 参数：`/home/akun/workspace/Car_jetson/nav2/src/nav2/config/nav2_real_params.yaml`
- 失效录包信息：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-16_16-34-22_manual_publish_monitor/bag_info.txt`
