# 第十四次独立自动充电实车测试

2026-11-24，用户将车辆放在 P0 并授权一次独立自动充电测试。记录器和 rosbag 先启动，独立功能包使用 `dry_run=false`、`calibrated=true`、`auto_start=false`；完整预检通过后只调用一次 `/standalone_docking/start`，没有自动重试。

## 结果

**本次未完成对接，也没有进入 `LOCK_VERIFY`。车辆在锁定距离内曾短暂准确对齐，但没有连续保持 0.5 秒；误差随后反向增大，完整桩体观测反复丢失，最终在距充电面约 0.79 m 处因 `dock_observation_reacquire_timeout` 停车。**

动作从 trace t=310.272 s 开始，t=389.872 s 客户端进入 FAILED，共约 79.60 秒。`LOCK_VERIFY` 和 `LOCKED_FINAL_APPROACH` 均进入 0 次，真实 `OBSERVATION_HOLD` 共 5 段。

| trace t / s | 事件与测量 |
|---:|---|
| 310.272 | DOCKING |
| 312.034 | APPROACH |
| 343.704 | FINAL_APPROACH |
| 365.426 | 距面约 1.150 m，结束预定位路径，随后从 0.025 m/s 切回 0.06 m/s |
| 367.865～368.149 | 同时满足 5 mm/0.5°门限，最长连续约 0.284 s |
| 368.065 | 最佳对齐：距面约 1.020 m，横向 0.047 mm、角度 0.016° |
| 369.280 | 距面约 0.960 m，横向已增至 7.49 mm，退出锁定门限并降至 0.025 m/s |
| 375.827 | 首次失观保持；距面约 0.806 m，横向约 16.9 mm、角度约 3.57° |
| 389.263 | STOPPING，`dock_observation_reacquire_timeout` |
| 389.823 | 控制器 FAILED 并发布紧停 |
| 389.872 | 独立客户端 FAILED |

物理面距离为 Action feedback 的接收面纵向误差加已确认的 0.46 m 接收面距离，属于基于当前标定的估算。

## 未进入锁定的原因

1. **稳定时间没有达到门限。** 锁定区内最好的对齐质量远好于 5 mm/0.5°要求，但最长连续片段只有约 0.284 s（8 个 Action feedback 样本），小于 `lock_alignment_duration=0.50 s`。因此控制器按设计继续运动，没有停车进入锁定验证。

2. **锁定区入口存在速度和目标切换。** 距面约 1.15 m 时，控制误差从 1.10 m 预定位虚拟目标切回最终接收面目标。此时纵向误差仍大于 `final_zone=0.50 m`，状态从 FINAL_APPROACH 回到 APPROACH，命令由 -0.025 m/s 提高到 -0.06 m/s。最佳对齐发生在该高速段；车辆继续约 1.2 秒后横向超过 5 mm。该切换是误差快速越过锁定窗口的重要诱因，但单次数据不能证明它是唯一原因。

3. **转向修正达到上限并产生过调。** 本轮 ECU 转向最大达到 8°配置上限。对齐前航向误差由负值修到接近零，随后继续变为正值；首次失观时已约 +3.57°，终止时约 +4.32°。横向误差也由接近零增至约 +24 mm。

4. **近距离完整轮廓关联不稳定。** 动作开始后共有 2255 条 `ok` 观测、24 条 `no_complete_face_in_gate` 和重置后的 60 条 `warming_up`。首次失观后虽然多次短暂重获，但无法连续满足 0.5 s 稳定重获，最终触发 2 s 局部重获时限。这发生在锁定之前，所以尚不能使用锁定位姿和里程计完成末段倒车。

5. **底盘低速分辨率和短暂回滚会缩短稳定窗口。** 最大轮速约 0.0813 m/s，最大左右差约 0.0521 m/s；停车/换 N 后最大正向平均轮速约 0.0159 m/s。全程没有正向 Twist 指令和 D 档，因此这些是实际低速反馈及短暂滚动，不是控制器要求前进。本轮没有轮速有效性或超速故障。

## 启动前发现并修复的问题

第一次预检未启动运动：IMU 驱动的设备时间戳在运行约一分钟后比主机 ROS 时间快约 0.35 s，短程里程计持续报告 `can_or_imu_stale`。独立硬件配置已加入 `use_device_timestamp_: false`，改用主机接收时间；重启后实测 IMU 时间差约 1 ms，4 秒预检中 200 个里程计状态全部为 `valid`。

修正后的预检确认自动驾驶 ON、主速度及双轮速度为零、N 档、单一 ECU 发布者、所有传感器新鲜；115 帧静态有效目标，充电面约 2.511 m，横向约 4.7 mm。

## 停车与数据

失败后另行接收 3 秒新鲜反馈：300 条车辆状态和 150 条 ECU 消息均确认主速度及双轮速度为零、N 档、motor=0。当前 JD03 转换器明确不再把普通零速或紧停映射为复用的 brake/EBrake 位，因此 `brake=false` 是当前接口行为，不能写成“制动位已确认”；车辆实测静止后才关闭节点。

驱动、控制器、记录器均已退出，ROS 节点列表为空。bag 正常关闭，共 202671 条消息、11490 帧后雷达，SQLite `quick_check=ok`。数据库只在 recorder 退出后读取。

- `prestart_check_before_imu_fix.json`：IMU 未来时间导致里程计无效的证据。
- `prestart_check.json`、`start_service.log`：修正后的通过检查和单次启动响应。
- `analysis.json`：状态、对齐片段、关键速度切换、观测及运动统计。
- `stop_confirmation.json`、`shutdown_confirmation.json`、`bag_integrity.json`：停车、退出和数据完整性。
- `motion_trace.jsonl`、`monitor.log`、`motion_summary.json`、`bag/`：原始记录。
- `relative_control_snapshot.py`、`feedback_odometry_snapshot.py`、`relative_docking.yaml`、`hardware.yaml`：本轮使用的代码和配置快照。
