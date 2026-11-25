# 2026-11-20 室内静态复测记录

## 结论

本轮证明正式根工作区 ZUPT、LIO 会话身份、scan-to-map correction、epoch 和连续性硬门限在室内静止条件下工作正常；没有复现此前“静止后 correction 持续增大并三连拒绝”的故障。规定的约 3 分钟窗口内 1149 次 correction 全部接受，最大平移 residual 1.83 cm；整段约 59 分钟 LIO 数据中 22990 次 correction 仅启动首帧因 scan-to-map skipped 被拒绝，之后 22989 次全部接受，最大 residual 2.29 cm。

本轮不能作为双重定位或完整重定位验收。没有启动 ProcessManager、重定位节点、Nav2、定位会话和 `map_odom_authority`，也没有地图先验或导航任务。特别是连续 `/odom` 在 3 分钟内累计约 3.07 m（主要是 Z 轴），而 `mapping/odometry` 同期首尾仅差 1.34 cm。当前架构有意保持 `odom` 连续、把全局 correction 交给 `map_odom_authority` 形成 `map→odom`；本轮缺少该生产消费者，因此不能用单独 `/odom` 的绝对值代表最终 `map→base_link`，但该累计 backlog 必须在下一轮实车中联合验证。

判定：

- 静态 scan-to-map 稳定性：通过。
- ZUPT 是否实际触发并降低预积分速度：通过。
- correction 三连失败是否复现：未复现。
- LIO/TF 相邻步长硬门限：通过。
- 静止最终全局位姿不漂移：本轮不具备生产 `map→odom` 链路，不能验收。
- 双重定位/重定位：本轮未测试，不能验收。

## 测试配置与范围

- 传感器：`ros2 launch launch_real run.launch.py`。
- LIO：根工作区 `/home/akun/workspace/Car_jetson/install/lio_sam`。
- 参数：`slam/src/LIO-SAM/config/params_rslidar.yaml`，`useVehicleZeroVelocityUpdate=true`。
- LIO 身份：`indoor_static_repeat_final_20261120161508`，generation `1`。
- observation-only 监控；未启用保护或终态自动退出。
- 未发布导航任务、`/cmd_vel`、`/process_manager/fault` 或其他车辆控制命令。
- 规定窗口：2026-11-20 16:15:42 至 16:19:00，约 198 秒。
- 因界面中断，后台只读采集延长；LIO 有效数据约 3561 秒（约 59.4 分钟），完整 bag 约 3645 秒（约 60.7 分钟）。

## 规定静态窗口指标

- correction：1149/1149 接受，拒绝 0；最大 0.01828 m，P95 0.00912 m，P99 0.01280 m；超过 0.05/0.10/0.15 m 均为 0。
- ZUPT：93 次，底盘速度最大 0.000 km/h；约束前速度最大 0.04261 m/s、P95 0.02994 m/s；约束后最大 0.03660 m/s、P95 0.02680 m/s；93 次均降低速度。
- `mapping/odometry`：首尾差 0.01344 m；XYZ 跨度约 0.0205/0.0262/0.0246 m。
- `odom_incremental`：首尾差 0.01701 m；XYZ 跨度约 0.0258/0.0203/0.0959 m，Z 方向存在启动瞬态。
- 连续 `/odom`：首尾差 3.073 m，主要为 Z 轴；相邻步长最大 0.000540 m，没有超过 0.05 m 硬门限。

## 整段长样本指标

- correction：22990 次，接受 22989，拒绝 1；唯一拒绝为启动首帧 `scan-to-map skipped`。最大 residual 0.02291 m，P99 0.01075 m，超过 0.05 m 为 0。
- `LIO_SCAN_TO_MAP_UNRELIABLE`：0。
- `LIO_ODOM_CONTINUITY_FAULT`：0。
- `missing_identity_or_epoch`：0。
- 定位 fault topic 消息：0。
- epoch：2 到 22990，倒退 0；全程仅一个 instance/generation。
- ZUPT：1687 次；1679 次降低速度，8 次轻微增加，最大增加仅 0.000434 m/s；底盘速度始终报告 0.000 km/h。加速度偏置范数最终为 0.04011。
- `mapping/odometry`：首尾差 0.01822 m，距起点最大 0.0400 m；XYZ 总跨度约 0.0559/0.0451/0.0379 m。
- `odom_incremental`：首尾差 0.02680 m，距起点最大 0.08247 m；XYZ 总跨度约 0.0506/0.0359/0.1178 m。
- `/odom`：相邻步长最大 0.000540 m，P99.9 0.000462 m，最大 yaw 步长 0.00203°；没有超过 0.05 m/1°。绝对首尾差 3.388 m，需由生产 `map→odom` 链路联合评估。
- `/odom_incremental`：相邻步长最大 0.01648 m，P99.9 0.00315 m；没有超过 0.05 m/1°。
- IMU/AHRS CRC：启动期 IMU 1 次，结束前 AHRS 1 次；未形成 CRC storm。

## 安全与落盘审计

- bag 原始大小 65.8 GiB，122 个 SQLite 分片，3,190,697 条消息，元数据完整。
- `/cmd_vel`：0 条。
- `/ecu`：0 条。
- `/process_manager/fault`：0 条。
- `/lio_sam/localization_fault`：0 条。
- 所有测试进程已停止；没有发布车辆控制或故障命令。

## 下一轮实车重点

1. 启动完整生产管理栈和 `map_odom_authority`，先静止 2～5 分钟，同时看 `odom→base_link`、`map→odom` 和最终 `map→base_link`，确认累计 backlog 能被限速跟随且最终全局位姿稳定。
2. 核对 `map→odom` backlog 最大值、跟随延迟和静态 footprint 门禁，不只看单帧 correction residual。
3. 再做双重定位/重定位任务；本轮室内静态数据不替代重定位验收。
4. 保持 15 cm/2° correction 硬拒绝和三连 rejected 故障门限，不因本轮通过而放宽。

## 保留文件

- `static_repeat_metrics.json`：规定窗口和整段聚合指标。
- `lio_bag_analysis.json`：整包 odometry、epoch、correction 详细分析。
- `lio_launch.log`、`sensor_launch.log`、`monitor.raw.jsonl`：运行日志和 observation-only 记录。
- `bag_metadata.yaml`、`bag_info.txt`：删除原始 bag 前保留的元数据和 topic 计数。

原始 `full_bag` 按操作者指令在完成上述分析和记录后删除。

## 原始 bag 删除记录

- 2026-11-20 17:33:06，将本轮唯一目标 `full_bag` 移入系统回收站。
- 核对回收站原始路径与本轮目录完全一致后，仅对 `trash:///full_bag` 执行永久删除；没有清空或修改其他回收站项目。
- 原始 65.8 GiB bag 及其 122 个 SQLite 分片已不可恢复。
- 本轮目录保留约 29 MiB 的日志、分析 JSON、报告、bag 元数据与 topic 计数。
- 删除后文件系统可用空间约 110.6 GB（`df -B1`），没有进程持有已删除的 bag 文件。
