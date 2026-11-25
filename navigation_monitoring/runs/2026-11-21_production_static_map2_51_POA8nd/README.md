# map2 航点51附近：生产定位链静止验证尝试及收尾

日期：2026-11-21，时间均为 Asia/Shanghai。

## 当前状态

- 用户最新要求为“保存录包用于后续分析，关闭定位链”。已执行；原始录包保留，未删除。
- 已停止本轮监控、录包及只读探针。
- 通过 `/process_manager/shutdown_process` 关闭 `re_localization`，返回成功。
- 向已确认的管理 launch 进程组30270发送SIGINT后，管理器、map_odom_authority及导航/定位子进程均退出；最终进程检查未见相关残留。
- 用户原有 `launch_real run.launch.py` 进程组29640及传感器/底盘驱动保留运行。
- 没有发布导航目标、车辆控制命令或 `/process_manager/fault`。本次记录的一条fault由生产ProcessManager在初始位姿进程退出后自行发布，不是测试程序发布。

## 验证范围与结果

原计划：按生产服务启动重定位、初始位姿、LIO、Nav2，全部就绪后静止观察300秒。

实际未达到生产链就绪：15:51:01初始重定位返回 `success=false`、`fitness_score=inf`、`cloud_stamp=0`，原因 `PSO returned no valid pose`。因此没有启动LIO和Nav2，没有绕过门禁、强行注入位姿或重复派单。

本次不能作为ZUPT、运行中扫描匹配或最终map→base_link静止验收。完整生产链就绪后的5分钟窗口没有开始。

首次测试程序在启动失败后自动收尾。之后补开故障上下文观察；两段之间存在采集间隔，不能称为连续5分钟。用户随后要求结束，第二段在123秒时停止。

## 保留录包

| 相对目录 | 开始 | 结束 | 时长 | 消息数 | SQLite分片 |
|---|---|---|---|---|---|
| `full_bag` | 15:50:54.791 | 15:51:11.466 | 16.674892秒 | 21,682 | 1 |
| `failure_observation/full_bag` | 15:53:15.279 | 15:55:18.303 | 123.023102秒 | 104,421 | 7 |

两段共126,103条消息，SQLite数据合计7,679,430,656字节（约7.15 GiB）。两段均正常SIGINT停止并生成metadata.yaml；8个分片只读 `PRAGMA quick_check` 均为ok，数据库实际消息数与元数据一致。逐分片及topic计数见 `closeout_check.json`。

同时保留运行脚本、服务响应、events.jsonl、snapshots.jsonl、result.json、monitor.raw.jsonl、recorder.log及相关只读观测结果。后续分析可直接使用原始bag，当前未压缩或清理。

## 初步故障线索（尚未实施修复）

对第一段bag的只读核查发现：

- `/pcl_localization/transition_event` 只有unconfigured→configuring→inactive两条，没有active。
- 节点日志没有Activating、Map Size、Initial Map Published等地图激活记录。
- 15:50:56.046存在 `/pcl_localization/change_state` 响应timeout日志。
- 当前源码只有on_activate才装载PCD并设置地图就绪；地图未就绪时服务直接返回，分数维持inf。
- 同一bag包含107帧 `/rslidar_Pointcloud2` 和17条实际使用的GNSS pose消息，不能直接归为雷达/GPS无输入。
- 当前ProcessManager对重定位的启动成功判据仅检查进程存活，尚不足以证明Lifecycle ACTIVE、地图已加载。

因此直接阻断点为重定位未进入激活/地图就绪阶段；为什么自动激活没有完成，仍需后续诊断。当前证据不支持归因于IMU漂移、配准质量或地图文件损坏。本轮没有修改生产代码。
