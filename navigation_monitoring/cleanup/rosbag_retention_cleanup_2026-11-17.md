# ROS 2 录包问题留存与清理记录（2026-11-17）

记录时间：2026-11-17T22:17:55+08:00（北京时间）。

## 留存边界与范围

- 用户指定从上周三开始保留；本次换算后的留存起点为 **2026-11-11 00:00:00+08:00**。
- 删除录制开始时间早于该时刻的实际 ROS 2 录包；保留 2026-11-11 当天及之后的录包。
- 删除对象共 **26 组**：导航录包 15 组、充电桩诊断录包 11 组，共 **194 个 `.db3` 分片**，没有 `.mcap`。
- 删除对象的表观文件大小约 96,629,792,984 B；连同 SQLite 辅助文件和目录开销，实际磁盘占用 **96,630,693,888 B（89.9944 GiB）**。
- 删除前未发现运行中的 `ros2 bag record`，目标文件没有硬链接。
- `handoffs/.../snapshots` 中另有 8 份不含 `.db3`/`.mcap` 的元数据快照；它们不是实际录包载荷，作为历史证据保留，不计入本次 26 组。
- 只删除下表中的 bag 目录；外围日志、监控 JSONL、分析结果、代码/配置快照和诊断报告全部保留。

## 删除前完整性检查

1. 26 组中有 25 组包含 `metadata.yaml`，对全部 194 个分片完成了只读表结构、消息数及首末时间轻量检查；除下述坏库外，193 个分片均可读，每组数据库消息总数都与 metadata 一致，`relative_file_paths` 中列出的分片也都存在。全页 `quick_check` 在删除前只完成到 2026-11-10 14:57 的包，不能声称对全部旧包完成了全页扫描。
2. `2026-11-10_10-49-33/navigation_core_bag` 缺少 `metadata.yaml`，唯一数据库读取时报 `database disk image is malformed`。其约 44.70 MiB 数据不可正常查询，录制时间只能按目录名和文件修改时间估算为约 10:49:33–10:51:24。
3. `2026-11-07_16-17-52/navigation_raw_bag` 只有 5 条消息、持续约 115 微秒、约 24.95 KiB，属于近空录包。
4. `2026-11-07_16-25-58` 和 `20261105_diagnostic_run10/bag` 的 metadata v5 中 `files: []` 为空，但 `relative_file_paths`、聚合统计和实际数据库均正常；这是元数据明细缺失，不是载荷丢失。其余 22 组未发现消息计数、分片清单或轻量可读性异常。
5. 所有带 metadata 的旧录包均未启用压缩（`compression_format` 和 `compression_mode` 为空）。空间分析已确认主要增量来自高频完整 `/global_costmap/costmap_raw` 和原始点云；两个最大旧包合计约 64.87 GiB。
6. `2026-11-07_16-25-58` 的 bag 只覆盖至 16:29:08.884，而对应人工停止约在 16:30:43，最后约 94 秒没有进入 bag；因此它不能复原用户所报偏离的完整末段。

## 已从旧录包提取并保留的问题

### 导航与定位

- 2026-11-07 两次异常重启被确认是整机非正常中断，但现存日志没有保存 kernel panic、OOM、过热关机或正常关机尾迹；PCIe Realtek 网卡链路存在持续可纠正 RxErr，只能作为线索，根因仍未定案。
- 16:36 初始化失败链已明确：重定位节点没有进入 ACTIVE、地图未装入，管理器却把“进程仍存活”误判为“服务已就绪”；PSO 随后以 `inf` 占位结果失败。
- 11 月 7 日导航段记录到约 2.164 m 的早期横向偏摆。LIO-SAM 把导航坐标系速度直接写入 `/odom.twist`，Nav2/MPPI 又把 `linear.x` 当机体系前向速度使用，存在确定的速度坐标系集成错误。
- 113 航点的 `yaw_goal_tolerance=6.28` 基本不约束朝向；车辆在未对齐下一路段时即完成航点，随后带着约 23° 的方向差进入 106 段，放大了初段绕弯。
- costmap 中“缓冲区内有效点数为 0”是确定的调试计数器误报警，不能据此断言雷达数据实际丢失。
- 2026-11-10 106→107 的全局规划反复失败，首要原因已定位为 Lakibeam 驱动 `inverted=true` 后，合并器又启用 `flip1/flip2=true`，二次反转把路侧纵向边界错误投影为车前横墙。2.0 m 全局膨胀、全局/局部 footprint 不一致和转弯半径不一致进一步放大失败。

### 充电对接

- run06：末段倒车指令存在，但实际轮速接近零，局部时限前仍距充电面约 0.92 m；记录不足以区分底盘低速映射、制动释放或机械原因。
- run07：再次出现极弱低速响应，总时限不足；不是碰撞急停、锁距计算失败或完整桩体失观造成。
- run08：N 档制动滑行时，反馈积分器忽略已有符号轮速并报 `moving_without_direction_feedback`，触发紧停；冻结的 TF/速度估计随后导致 `stop_not_confirmed`。修复只完成离线回放，未在该轮重新闭环实车验证。
- run09：桩位锁定成功，但锁定后 35 秒只倒退约 0.198 m，最终 `locked_final_timeout`；紧停发生在 FAILED 之后，不是首因。
- run10：由扫掠碰撞检查停车，但精确触发帧缺失，无法断言障碍物身份；停车后的短暂正轮速不是先触发原因。
- run11：距充电面约 1.08 m 时出现瞬时左右轮速异常并锁存 `invalid_wheel_feedback`；没有原始 CAN 帧，无法继续区分轮滑、底盘上报或解码通信问题。
- run12：未复现 run11 的轮速差异常；在约 0.76 m 处因观测无法在 2 秒内稳定重获而 `dock_observation_reacquire_timeout`，尚未进入锁定验证。
- run13：在约 0.79 m 处因目标无法持续稳定重获而停车，同样未进入锁定验证；紧停为失败后的收尾，不是首因。
- run03–run05 没有找到独立的派生诊断报告；仅保留了本清理记录中的时间、大小和完整性结果。

上述结论的派生报告、监控轨迹和 JSON 分析仍保留在原 run 外围目录。关键入口包括：

- `navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/RESULT.md`
- `navigation_monitoring/runs/2026-11-07_16-25-58/deviation_analysis_bag.md`
- `navigation_monitoring/runs/2026-11-07_16-25-58/deviation_analysis_costmap.md`
- `navigation_monitoring/runs/2026-11-07_16-25-58/speed_change_analysis.md`
- `navigation_monitoring/runs/2026-11-10_11-04-54/global_planner_failure_analysis.md`
- `test_data/charging_dock/20261105_diagnostic_run06` 至 `20261110_diagnostic_run13` 各自的 `README.md`、`analysis.json`、`fault_window.json` 和 `motion_trace.jsonl`

## 删除清单

| 录包目录（相对 `Car_jetson/`） | 录制时间（北京时间） | 大小 |
|---|---|---:|
| `navigation_monitoring/runs/2026-11-07_16-17-52/navigation_raw_bag` | 2026-11-07 16:20:12，约 115 微秒 | 24.95 KiB |
| `navigation_monitoring/runs/2026-11-07_16-25-58/navigation_raw_bag` | 16:25:58–16:29:08 | 337.71 MiB |
| `navigation_monitoring/runs/2026-11-07_16-36-44/navigation_raw_bag` | 16:36:44–16:39:25 | 138.04 MiB |
| `navigation_monitoring/runs/2026-11-07_17-03-42/navigation_raw_bag` | 17:03:42–17:09:27 | 251.69 MiB |
| `navigation_monitoring/runs/2026-11-09_14-33-18/navigation_core_bag` | 2026-11-09 14:38:44–14:45:32 | 5.03 GiB |
| `navigation_monitoring/runs/2026-11-09_15-47-22/navigation_core_bag` | 15:47:23–15:53:58 | 4.49 GiB |
| `navigation_monitoring/runs/2026-11-10_10-17-53/navigation_core_bag` | 2026-11-10 10:17:55–10:45:41 | 41.39 GiB |
| `navigation_monitoring/runs/2026-11-10_10-49-33/navigation_core_bag` | 约 10:49:33–10:51:24；损坏 | 44.70 MiB |
| `navigation_monitoring/runs/2026-11-10_10-59-56/navigation_core_bag` | 10:59:56–11:02:16 | 1.54 GiB |
| `navigation_monitoring/runs/2026-11-10_11-04-54/navigation_core_bag` | 11:04:54–11:19:58 | 23.49 GiB |
| `navigation_monitoring/runs/2026-11-10_14-57-42/navigation_core_bag` | 14:57:44–15:05:36 | 3.96 GiB |
| `navigation_monitoring/runs/2026-11-10_15-25-10/navigation_core_bag` | 15:25:12–15:28:15 | 1.72 GiB |
| `navigation_monitoring/runs/2026-11-10_15-51-41/navigation_core_bag` | 15:51:42–15:55:46 | 2.19 GiB |
| `navigation_monitoring/runs/2026-11-10_16-36-32/navigation_core_bag` | 16:36:33–16:39:49 | 2.07 GiB |
| `navigation_monitoring/runs/2026-11-10_16-42-07/navigation_core_bag` | 16:42:08–16:45:35 | 1.96 GiB |
| `test_data/charging_dock/20261104_diagnostic_run03` | 2026-11-04 16:58:49–17:08:07 | 202.08 MiB |
| `test_data/charging_dock/20261104_diagnostic_run04` | 17:42:18–17:47:26 | 140.96 MiB |
| `test_data/charging_dock/20261104_diagnostic_run05` | 18:32:46–18:35:14 | 66.15 MiB |
| `test_data/charging_dock/20261105_diagnostic_run06/bag` | 2026-11-05 12:01:52–12:05:38 | 100.58 MiB |
| `test_data/charging_dock/20261105_diagnostic_run07/bag` | 14:06:02–14:10:05 | 105.56 MiB |
| `test_data/charging_dock/20261105_diagnostic_run08/bag` | 14:26:51–14:30:21 | 89.28 MiB |
| `test_data/charging_dock/20261105_diagnostic_run09/bag` | 16:30:36–16:35:50 | 147.03 MiB |
| `test_data/charging_dock/20261105_diagnostic_run10/bag` | 17:03:21–17:06:51 | 98.29 MiB |
| `test_data/charging_dock/20261106_diagnostic_run11/bag` | 2026-11-06 12:05:13–12:09:03 | 108.59 MiB |
| `test_data/charging_dock/20261106_diagnostic_run12/bag` | 12:44:33–12:48:19 | 106.36 MiB |
| `test_data/charging_dock/20261110_diagnostic_run13/bag` | 2026-11-10 13:55:48–14:07:05 | 319.55 MiB |

## 清理结果

完成时间：2026-11-17T22:23:18+08:00。

- 上表 26 个录包目录及其中 194 个 `.db3` 分片已永久删除；删除不可恢复。
- 删除前目标实际占用 96,630,693,888 B（89.995 GiB）。按删除前后 `df` 的可用空间差计算，实测净增加约 96,625,606,656 B（89.990 GiB）；数 MiB 差异来自同期文件系统写入/记账。
- 文件系统可用空间由 46,047,637,504 B（42.89 GiB，使用率 91%）增至 **142,673,244,160 B（132.875 GiB / 142.673 GB，使用率 71%）**。
- 文件系统总容量 502,392,610,816 B（467.890 GiB），清理后已用 334,123,995,136 B（311.177 GiB）。
- 复核后剩余 **17 组**实际 ROS 2 录包、436 个 `.db3` 分片、0 个 `.mcap`；全部录制于 2026-11-11 当天或之后，没有截止线前的 ROS 2 数据分片残留。
- 两个抽样证据入口及本记录均已确认存在，外围诊断、日志、分析结果和监控轨迹未删除。
