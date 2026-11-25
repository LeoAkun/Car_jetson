# 2026-11-18 实车测试收尾摘要

## 停止结果与控制边界

- 12:13:48.916 收到“结束测试”，12:13:49.172 完成收尾。
- 在核对 boot ID、进程启动 tick 和 PGID 后，仅向当前第三段 recorder 与本轮只读 monitor 进程组发送 SIGINT。
- 两个进程均正常退出，无本轮残留进程；未停止导航或传感器，未发送任何车辆控制命令。
- 操作员未提供最终位置或人工结果，本报告只陈述日志证据。

## 录包概况

本轮按人工指令形成三个独立录包段：

| 分段 | 覆盖时间 | 时长 | 分片 | 消息 | 大小 |
|---|---|---:|---:|---:|---:|
| 1 | 11:25:26.195–11:33:48.219 | 502.023 秒 | 17 | 167,851 | 6.2 GiB |
| 2 | 11:35:33.535–11:40:02.911 | 269.376 秒 | 9 | 90,975 | 3.3 GiB |
| 3 | 11:52:39.174–12:13:48.967 | 1269.793 秒 | 60 | 528,061 | 42.5 GiB |
| 合计 | 三段 | 2041.192 秒（34分01秒） | 86 | 786,887 | 52.053 GiB |

两次人工停录形成约 861.58 秒（14分21.58秒）的总缺口。三段日志均未报告缓存丢失或 recorder 错误。

## 完整性

- 三段 metadata 中的分片集合均与磁盘文件完全一致。
- 全部 86 个 SQLite 分片以只读方式通过 `PRAGMA quick_check`。
- SQLite 消息数合计与 metadata 一致。
- 测试结束后磁盘剩余约 123 GiB。

## 关键话题计数

| 话题 | 三段合计消息数 |
|---|---:|
| `/odom_incremental` | 48,511 |
| `/odom` | 48,481 |
| `/lio_sam/mapping/odometry` | 3,670 |
| `/lio_sam/mapping/odometry_incremental` | 3,620 |
| `/imu` | 203,645 |
| `/rslidar_Pointcloud2` | 16,511 |
| `/tf` | 7,884 |
| `/vehicle_status` | 174,803 |
| `/cmd_vel` | 6,258 |
| `/navigate_to_pose/_action/feedback` | 20,731 |
| `/process_manager/fault` | 3 |

第一、二段没有发现 `/odom_incremental` 和 `/odom`；第三段已成功录到这两路关键融合话题。因此完整融合链分析应优先使用第三段，前两段不能用于逐帧证明 TransformFusion 输入/输出关系。

## 初步任务与异常结论

- 监测到 6 个唯一任务 ID，没有一个具备干净的全成功 terminal 证据。
- 11:29:10 和 11:38:08 两次出现 LIO-SAM 启动健康检查超时，缺少 `lio_sam_imuPreintegration` 子进程，任务进入 fault。
- 11:58:32 出现 `nav2_init_pose: launch process exited with code 2`，随后进入 fault。
- 11:53、11:59、12:06 开始的三次任务均出现 action `6=ABORTED`，随后回到 idle，不能认定导航任务成功。
- 传感器监测峰值：IMU 约 100.2 Hz，合并雷达约 30.7 Hz；最高温度 49°C，最低可用内存约 22.95 GiB。
- 监测到的最高车速为 4.4 km/h。

最后快照（12:13:46.222）为 robot idle/status=0、action status `[6]`，`cmd_vel=[0,0]`，但 `/cmd_vel`、`/odom`、`/tf` 已分别约 65.67/58.33/57.39 秒没有新消息；同时 `/vehicle_status` 报告 4.4 km/h。由于控制和里程计数据已陈旧，不能仅凭该快照判断现场车辆是否已经物理停止，现场安全状态仍由操作员确认。监测器未采取控制动作。

## 校验材料

- `bag_info.segment_1.txt`、`bag_info.segment_2.txt`、`bag_info.segment_3.txt`
- `sqlite_quick_check.segment_1.txt`、`sqlite_quick_check.segment_2.txt`、`sqlite_quick_check.segment_3.txt`
- `bag_record.log`、`bag_record.segment_2.log`、`bag_record.segment_3.log`
- `monitor.raw.jsonl`
