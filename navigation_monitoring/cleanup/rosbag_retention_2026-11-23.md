# 2026-11-23 录包清理记录

按操作者指令删除 8 个整包，并裁剪 11 月 17 日与 11 月 19 日下午两个录包。时间均为北京时间。

## 删除的整包

| 录包目录（相对 navigation_monitoring） | 实际占用 |
|---|---:|
| `runs/2026-11-21_production_static_map2_51_POA8nd/failure_observation/full_bag` | 6.33 GiB |
| `runs/2026-11-20_15-54-05_indoor_static_repeat/full_bag` | 29.77 MiB |
| `simulation/2026-11-19_1328_current_changes/runtime/lio_replay_output` | 36 KiB |
| `simulation/2026-11-19_1328_current_changes/runtime/lio_replay_output_current` | 36 KiB |
| `simulation/2026-11-19_1328_current_changes/runtime/lio_replay_output_current_q10` | 36 KiB |
| `simulation/2026-11-19_1328_current_changes/runtime/lio_replay_output_current_clean_player` | 36 KiB |
| `runs/2026-11-14_11-38-38_manual_publish_monitor/navigation_core_bag` | 4.82 GiB |
| `runs/2026-11-18_11-25-25_manual_publish_monitor/navigation_core_bag_segment_2` | 3.35 GiB |

整包释放约 14.53 GiB。11 月 21 日 15:50 的失败原包仍在。11 月 18 日第二段的 45 秒回放片段仍在 `simulation/2026-11-19_1328_current_changes/runtime/sensor_window_45s_current`。该片段通过 SQLite 检查，4,856 条 IMU、点云和静态 TF 消息的时间戳和载荷均与原包一致。

## 部分删除

| 运行目录 | 原占用 | 保留占用 | 保留分片 | 保留消息 | 删除分片 | 释放空间 |
|---|---:|---:|---:|---:|---:|---:|
| `2026-11-17_15-44-58_manual_publish_monitor` | 120.58 GiB | 112.20 GiB | 137 | 1,270,855 | 27 | 8.38 GiB |
| `2026-11-19_16-55-28_manual_publish_monitor` | 17.54 GiB | 5.83 GiB | 16 | 156,109 | 33 | 11.71 GiB |

两份裁剪后的包仍在各自的 `navigation_core_bag/`。删除的分片没有进入回收站。

| 日期 | 保留时段 | 内容 |
|---|---|---|
| 11-17 | 15:44:59–15:45:29 | 静态 TF、录包启动 |
| 11-17 | 15:46:29–15:51:14 | 跨图路线和任务状态 |
| 11-17 | 15:52:44–16:00:05 | 换图、Nav2 组件加载失败与恢复 |
| 11-17 | 16:02:05–16:04:35 | 第一次失败重定位启动 |
| 11-17 | 16:08:05–16:20:41 | 第一次重定位失败结尾、第一段实际行驶、倒车、备用路线及第二次失败重定位启动 |
| 11-17 | 16:24:11–16:44:16 | 第二次重定位失败结尾、Nav2 启动超时、第二段实际行驶、倒车及备用路线 |
| 11-19 | 16:55:28–16:55:58 | 静态 TF、录包启动 |
| 11-19 | 17:04:28–17:10:28 | 首任务、定位保护触发、停止与管理器退出 |
| 11-19 | 17:11:28–17:12:58 | 第二次任务及故障状态 |

按原始分片边界裁剪，保留时段之间存在时间缺口。各运行目录的 `bag_metadata_before_retention_2026-11-23.yaml` 记录原包，`bag_retention_2026-11-23.json` 列出保留和删除的分片、实际时段与校验结果。

11 月 17 日原索引中有 14 个分片的计数或结束时间与 SQLite 记录不符，相关分片通过 SQLite 快速检查。新索引使用实际计数和时间戳。复核确认两包原有的静态 TF、任务状态、路线和故障消息均保留，11 月 19 日原有的 LIO、里程计及速度指令消息也全部保留。两个保留包都通过 ROS2 索引和跨时段读取检查。

整包删除及裁剪总计释放约 34.62 GiB。
