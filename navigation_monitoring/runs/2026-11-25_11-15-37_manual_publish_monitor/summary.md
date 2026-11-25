# 2026-11-25 实车测试收尾

11:53:31.251 收到结束指令，11:53:31.764 确认本轮只读监测和录包均已退出。SIGINT 发给本轮已核验的监测和录包进程组。结束指令未附现场位置或人工测试结论。

## 录包

录包覆盖 11:15:37.772–11:53:31.307（北京时间），时长 37 分 53.535 秒，共 83 个 SQLite 分片、49.42 GiB、1,185,281 条消息。全部分片通过只读 `PRAGMA quick_check`，文件清单、总消息数和逐话题计数均与 metadata 一致。录包日志未报告缓存丢失或写入错误。

| 关键话题 | 消息数 |
|---|---:|
| `/imu` | 227,356 |
| `/rslidar_Pointcloud2` | 19,526 |
| `/odom_incremental` | 71,557 |
| `/odom` | 71,257 |
| `/lio_sam/mapping/odometry` | 2,322 |
| `/lio_sam/mapping/odometry_incremental` | 2,320 |
| `/tf` | 71,572 |
| `/rosout` | 7,835 |
| `/lio_sam/correction_event` | 1,853 |
| `/lio_sam/localization_fault` | 5 |
| `/process_manager/fault` | 1 |

## 任务和异常

共收到 7 个任务。录包确认 1 个任务到达全部航点，另外 6 个进入 `fault`。时间为录包的消息接收时间。

| 时间 | 地图与航点 | 观察结果 |
|---|---|---|
| 11:19:13.330 | `3b2mus` 4→7→8 | 已到达 4，前往 7 时连续 3 次拒绝扫描匹配修正，报 `LIO_SCAN_TO_MAP_UNRELIABLE` |
| 11:26:42.107 | `3b2mus` 7→8 | 到达两个航点，随后回到 `idle` |
| 11:37:56.291 | `map2` 20→21→22→23 | 前往 20 时报告 `LIO_ODOM_CONTINUITY_FAULT`（epoch 231） |
| 11:40:46.108 | `map2` 20→21→22→23 | 前往 20 时报告 `LIO_ODOM_CONTINUITY_FAULT`（epoch 451） |
| 11:43:07.237 | `map2` 20→21→22→23 | 前往 20 时报告 `LIO_ODOM_CONTINUITY_FAULT`（epoch 151） |
| 11:46:13.994 | `map2` 20→21→22→23 | 到达 20 后，前往 21 时报告 `LIO_ODOM_CONTINUITY_FAULT`（epoch 437） |
| 11:51:30.367 | `map2` 15→16→17 | `nav2_init_pose` 以退出码 2 结束 |

四次里程计连续性故障的原因仍需结合录包的修正事件、增量里程计及 TF 复盘。

结束前最后一帧（11:53:26.761）报告机器人状态为 `fault`、车速 4.5 km/h，自动驾驶标志为 `true`。停稳状态需现场确认。
