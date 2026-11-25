# 2026-11-23 下午实车测试收尾

17:13:12.428 收到结束指令，17:13:12.689 确认只读监测与录包均已退出，无本轮进程残留。只向两个已核验的进程组发送 SIGINT。现场位置和人工测试结论未随结束指令提供。

## 录包

录包覆盖 16:40:28.956–17:13:12.437（北京时间），时长 32 分 43.481 秒。共 77 个 SQLite 分片，约 43.49 GiB，864,119 条消息。所有分片通过只读 `PRAGMA quick_check`，文件列表、总消息数及逐话题计数与 metadata 一致。录包日志未报告缓存丢失或 recorder 错误。

| 关键话题 | 消息数 |
|---|---:|
| `/imu` | 195,489 |
| `/rslidar_Pointcloud2` | 15,455 |
| `/odom_incremental` | 54,236 |
| `/odom` | 48,759 |
| `/lio_sam/mapping/odometry` | 3,562 |
| `/lio_sam/mapping/odometry_incremental` | 3,558 |
| `/tf` | 12,720 |
| `/rosout` | 6,814 |
| `/lio_sam/correction_event` | 2,704 |
| `/lio_sam/epoch_odometry` | 54,234 |
| `/lio_sam/map_odom_correction` | 49,104 |
| `/lio_sam/localization_fault` | 4 |

## 任务与故障

8 个任务均在 `map2`，记录中未见导航目标成功终态。下表时间来自本轮录包的故障话题。

| 时间 | 航点 | 故障记录 |
|---|---|---|
| 16:43:50 | 13→17 | 连续 3 次拒绝 scan-to-map 修正，报 `LIO_SCAN_TO_MAP_UNRELIABLE` |
| 16:47:07 | 17 | `nav2_init_pose` 以退出码 2 结束 |
| 16:51:06 | 17 | 地图位姿修正被拒绝，扩展车身轮廓覆盖静态占用格 (2009, 2758) |
| 16:54:54、16:57:32 | 15→16→17 | 两次 `nav2_init_pose` 以退出码 2 结束 |
| 17:02:08 | 20→21→22 | Nav2 启动健康检查超时，缺少 8 个 ROS 节点 |
| 17:04:33 | 20→21→22 | 连续 3 次拒绝 scan-to-map 修正，报 `LIO_SCAN_TO_MAP_UNRELIABLE`，随后 action 进入 `CANCELED` |
| 17:11:28 | 21→22 | 地图位姿修正被拒绝，扩展车身轮廓覆盖静态占用格 (2911, 4554) |

最后一次任务中，规划器于 17:11:13.966 报告 `Starting point in lethal space`，监测于 17:11:25.179 记录 `fault`，录包于 17:11:28.185 收到足迹校验拒绝。先后顺序基于各消息接收时间，因果关系仍需对齐位姿、TF 和代价地图确认。最后一个目标的 bag 状态消息止于 `EXECUTING`，只读监测另记录到 `CANCELING`。

结束前最后一帧（17:13:08.010）报告机器人状态为 `fault`，车速 0 km/h，自动驾驶标志为 false。
