# 实车测试收尾摘要

## 控制边界与停止结果

- 本轮监测为只读模式：`protection_enabled=false`、`stop_on_terminal=false`。
- 未发布任何车辆控制命令，未启动或停止导航任务，未停止导航或传感器进程。
- 2026-11-17 16:46:08.370+08:00 收到人工结束请求。
- 在核对 boot ID、进程启动 tick 和 PGID 后，仅向本轮保存的 monitor 与 rosbag 进程组发送 SIGINT。
- monitor 与 rosbag 均正常退出；rosbag 于 16:46:08.444 输出 `Recording stopped`，16:46:08.800 确认无本轮残留进程。

## 录包概况

- 目录：`navigation_core_bag/`
- 覆盖：2026-11-17 15:44:59.105 至 16:46:08.409（61 分 09.304 秒）
- 格式：SQLite3，未压缩
- 分片：164 个 `.db3`
- 大小：129,471,598,592 bytes（约 120.6 GiB）
- 已录消息：1,498,533 条
- 元数据中的分片集合与磁盘文件完全一致。
- 164 个 SQLite 分片均通过 `PRAGMA quick_check`；分片消息数合计与 metadata 一致。

关键话题已录消息数：

| 话题 | 消息数 |
|---|---:|
| `/odom_incremental` | 156,686 |
| `/odom` | 156,660 |
| `/lio_sam/mapping/odometry` | 9,688 |
| `/lio_sam/mapping/odometry_incremental` | 9,564 |
| `/imu` | 356,468 |
| `/vehicle_status` | 259,893 |
| `/cmd_vel` | 18,942 |
| `/cmd_vel_nav` | 2,167 |
| `/tf` | 23,582 |
| `/navigate_to_pose/_action/feedback` | 56,318 |
| `/process_manager/fault` | 6 |

`/gps/filtered`、`/odometry/navsat`、`/odometry/gpsz` 和 navigate-through-poses action 话题为 0 条；这是测试期间没有收到这些发布，不等同于录包程序故障。

## 数据完整性限制

rosbag 正常收尾，但写入缓存累计丢失 14,194 条消息。按“已录 + 已报告丢失”计算，丢失约占 0.9383%。两段集中窗口为：

- 16:15:26–16:17:41：累计丢失 7,861 条。
- 16:40:44–16:43:18：新增丢失 6,333 条，最终累计 14,194 条。

关键丢失量：`/imu` 2,451，`/odom_incremental` 2,344，`/odom` 2,342，`/vehicle_status` 1,621，导航 action feedback 1,226，`/cmd_vel` 457，`/tf` 173，LIO mapping odometry 126，LIO mapping incremental odometry 125。

因此，录包可读取、结构一致，但不能称为无丢帧完整录包；尤其不能对上述两个窗口作逐帧完整性保证。`/odom_incremental` 的估算丢失比例约 1.474%。

## 只读监测到的任务与异常

- 共观察到 9 个唯一任务 ID；没有一个具备“全程成功且 terminal 状态干净”的证据。
- 唯一 `monitor_terminal` 出现在 15:50:50.152，结果为 idle，但 action 状态中同时出现 `6=ABORTED`，不能据此认定整项任务成功。
- 15:59:17 与 16:31:37 两次出现 Nav2 启动健康检查超时，缺少 smoother、planner、behavior、bt_navigator、waypoint_follower、velocity_smoother 以及两个 lifecycle manager。
- 16:09:26 与 16:27:04 两次出现 `nav2_init_pose` 进程以 code 0 退出却进入 fault 的记录。
- 16:02:56 与 16:19:15 开始的任务期间未建立有效的 LIO/odom/tf 流，之后均进入 fault。
- 其余若干任务的 action 状态包含 `ABORTED`。任务 ID 切换只能证明收到了新的 waypoint list，无法仅凭日志区分操作员替换、续发或内部重发。

传感器监测中，IMU、合并雷达和过滤雷达从 15:45:04 至 16:46:07 始终有消息，典型频率约为 100 / 29.67 / 29.67 Hz；高负载时瞬时估计最低约为 57.83 / 22.39 / 22.39 Hz。

最后快照（16:46:07.143）为 robot idle/status=0、车速 5.0 km/h、drive_mode=3、is_autodrive=true；当时 `/cmd_vel`、`/odom`、`/tf` 已分别约 139 秒没有新消息，监测器缓存的最后一个 `cmd_vel` 是陈旧值，不能解释为当前控制命令。监测器只读，未对此采取控制动作。

## 尚未获得的人工结论

- 操作员未在本轮结束指令中提供最终任务结果或停车位置。
- 本摘要中的“无全成功证据”是日志证据结论，不等同于替操作员判定现场安全状态。

## 校验材料

- `bag_info.txt`：rosbag 元数据与逐话题计数
- `sqlite_quick_check.txt`：164 个分片的 SQLite 快速完整性检查
- `bag_record.log`：录包启动、分片、缓存丢失与正常停止日志
- `monitor.raw.jsonl`：只读监测快照和事件
