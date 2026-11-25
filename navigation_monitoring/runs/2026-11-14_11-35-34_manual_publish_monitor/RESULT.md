# 2026-11-14 手动派单测试：初始化尝试

本目录记录任务 `task_1794627438` 的第一次启动尝试。任务由现场手动发布，地图为 `map2`，航点为 `53 → 54 → 58 → 57`。

- 11:37:18：捕获任务，`robot_state=running`、`task_status=1`。
- 11:37:21：重定位和 `nav2_init_pose` 进程出现。
- 11:37:25：进程管理器报告 `nav2_init_pose: launch process exited with code 0`，机器人进入 `fault`。
- 11:37:26：相关子进程退出；11:37:27 后车辆保持 0 km/h、模式 3。

本次尝试未启动 NavigateToPose，车辆未移动，也未到达航点 53。rosbag 正常关闭，时长约 129.2 秒，共 49,264 条消息，约 128 MiB。系统随后自动重试，后续记录位于 `../2026-11-14_11-38-38_manual_publish_monitor/`。
