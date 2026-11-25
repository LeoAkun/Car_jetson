# 2026-11-10 15:25:10 实车测试结果

## 测试结果

- 地图与路线：`3b2mus`，113 → 106 → 107。
- 任务 ID：`task_1794295542`。
- 任务在 15:27:07 进入 `fault`，未发出有效导航速度，车辆全程 0 km/h。
- boot ID 未变化，没有整机重启。
- 未自动重试，未远程切换驾驶模式，未主动发布底盘制动。

## 本轮已验证的修复

1. RViz 已成功连接桌面会话，OpenGL 初始化正常，并加载 4097×6354 的地图；
   本轮没有 xcb 无法连接 DISPLAY 的错误。
2. LIO-SAM 在 15:26:43 实际遇到“没有比最新激光里程计更新的 IMU 里程计”条件；
   新增的空队列保护告警并返回，`imuPreintegration` 保持存活，没有再次触发
   `basic_string::_M_construct null not valid`。
3. 本轮未出现 11 月 9 日的 GTSAM `b0` 异常。该历史问题仍未因此被宣告修复。

## 本轮故障

Nav2 从 15:26:06 启动。10 个组件均已被加载，但大地图冷启动下组件装载和生命周期
配置/激活速度异常缓慢。60 秒健康检查结束时：

- controller、smoother、planner 为 inactive；
- behavior 查询超时；
- bt_navigator、waypoint_follower、velocity_smoother 仍 unconfigured。

因此管理器终止本轮任务。RViz 不是该超时的根因：上轮 RViz 因无 DISPLAY 早已退出时，
Nav2 仍发生相同的慢启动。历史成功轮的 Nav2 约 18 秒完成，本轮组件装载本身已耗时约
24.5 秒，随后生命周期配置仍未在 60 秒内完成。CPU 峰值约 41%，可用内存 26 GB 以上，
没有整机资源耗尽证据。

## 已实施的后续修复

- 保留 `max_startup_time=60s` 作为 LIO-SAM 等进程的启动上限。
- 新增 `nav2_max_startup_time=150s`，只允许 Nav2 大地图冷启动使用更长的受限窗口。
- Nav2 等待期间持续确认四个必需 LIO-SAM 子进程健康；一般缺失需连续 3 次确认，
  进程退出等明确故障则立即确认，并使 Nav2 启动失败，不等待满 150 秒。
- 相关环境、超时选择和 LIO 联锁测试共 18 项通过；`py_compile` 与
  `git diff --check` 通过。安装配置为源码链接，下次新管理进程直接加载修复。

此修改解决的是已观察到的 60 秒冷启动误判，并保留定位子栈失败时的快速中止；
它不证明 Nav2 非确定性变慢的底层原因已经消失。另一个已经定位的机制是：
生命周期管理器只会在全部 composable nodes 加载完成后才启动，而 controller、planner
等节点的 `on_configure()` 需要创建大尺寸 costmap；因此组件串行装载变慢与大地图 costmap
配置串联，容易把总启动时间推过 60 秒。

## 收尾状态

- rosbag 已正常封口：2 个 db3 分片，共 1,839,337,472 字节。
- 保护监测、cmd_vel→ECU 转换器、管理栈、Nav2、RViz、LIO-SAM、重定位均已退出。
- 原传感器驱动保持运行，参数仍为 `start_cmdvel_2_ecu:=false`。
- 未派发新的任务。

