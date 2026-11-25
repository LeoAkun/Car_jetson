# 2026-11-19 实车测试收尾摘要

## 停止结果与控制边界

- 17:19:33.059 收到“结束测试”，17:19:33.333 完成停止。
- 核对 boot ID、进程启动 tick 和 PGID 后，仅向本轮 recorder 与 observation-only monitor 进程组发送 SIGINT。
- 两个进程均正常退出，无本轮残留进程；未停止导航或传感器，未发送车辆控制命令。

## 录包与完整性

- 覆盖：16:55:28.511–17:19:33.065，共 24分04.553秒。
- 49 个 SQLite 分片，18,836,860,928 bytes（约 17.5 GiB），459,978 条消息。
- recorder 日志未报告缓存丢失或错误。
- metadata 分片集合与磁盘文件一致。
- 全部 49 个分片以只读方式通过 `PRAGMA quick_check`，SQLite 消息数与 metadata 一致。

| 关键话题 | 消息数 |
|---|---:|
| `/odom_incremental` | 2,267 |
| `/odom` | 2,267 |
| `/lio_sam/mapping/odometry` | 71 |
| `/lio_sam/mapping/odometry_incremental` | 71 |
| `/imu` | 144,404 |
| `/rslidar_Pointcloud2` | 13,472 |
| `/tf` | 2,402 |
| `/vehicle_status` | 118,795 |
| `/cmd_vel` | 3 |
| `/navigate_to_pose/_action/feedback` | 0 |
| `/process_manager/fault` | 0 |

## 初步事件结论

- 监测到两个任务 ID，没有干净的全成功 terminal 证据。
- 第一项任务 `task_1795079150`（map2，航点 13–16）于 17:05:50 开始；Nav2 进程出现后，robot state 在 17:06:21 进入 fault，但 `/process_manager/fault` 没有对应原因消息。17:09:38 记录到 `stop_navigation_script: operator_stop`，随后 manager 丢失。
- 第二项任务 `task_1795079523` 于 17:12:03 收到相同四个航点，但之后未观察到 navigation manager 启动，机器人持续为 fault。
- 监测到 5 个不同 correction epoch：3 个小 correction 被接受；0.169966 m 和 2.703927 m 两个 scan-to-map correction 超过 0.15 m 平移限制并被拒绝。最大 2.704 m correction 没有被应用，这是重要的保护性诊断结果。
- IMU 峰值约 100.2 Hz，雷达扫描峰值约 30.2 Hz；监测到的最高车速为 4.7 km/h。

最后快照（17:19:28.589）中 robot state 为 fault、robot status=0，`/vehicle_status` 报告 2.5 km/h；但 `/cmd_vel`、`/odom`、`/tf` 已分别约 786.7/766.1/765.2 秒没有新消息，因此这些流无法用于确认结束时车辆是否物理停止。现场安全状态仍须以操作员确认为准；监测器未采取控制动作。

操作员未在结束口令中提供最终位置或人工测试结果。
