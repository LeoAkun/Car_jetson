# 删除双轮差值门限后的第十二次独立实车测试

2026-11-06，用户将车辆放回指定 P0 后授权一次测试。根代理唯一控制硬件；先启动 trace 和 rosbag，再启动独立包，`dry_run=false`、`calibrated=true`、`auto_start=false`。4 秒静态检查全部通过后，只调用一次 `/standalone_docking/start`。本轮不修改生产参数，没有自动重试。

## 结果

**未完成对接；尚未进入锁定验证，距充电面约 0.76 m 时，因目标观测在 2 秒内未能持续稳定重获而停车。** 软件终态为 `FAILED dock_observation_reacquire_timeout`。

安装代码已核对与源码一致：独立积分器不再校验左右轮速差；仍保留有效性、新鲜度、单轮上限及共享实际超速保护。新版锁定逻辑要求停车前不同新扫描连续对齐 0.5 秒；实际停车后独立验证并锁定，本轮没有达到此阶段。

静态预检确认主速和双轮速均为零、自动驾驶 ON、N 档制动、里程计有效、单一 ECU 发布者、传感器数据新鲜；115 帧有效静态目标，最后测得充电面约 2.515 m。

| trace t / s | 事件 |
|---:|---|
| 62.037 | WAIT_SENSORS |
| 62.587 | DOCKING |
| 64.323 | APPROACH |
| 97.009 | FINAL_APPROACH |
| 128.926 | 再次 FINAL_APPROACH，随后持续雷达纠偏 |
| 约 137.296 | 距面约 0.907 m，曾短暂满足横向及角度门限 |
| 148.688 起 | 失观保持变频繁，确认重获后恢复，尚未锁定 |
| 158.084 | 最后一次 OBSERVATION_HOLD |
| 160.082 | STOPPING，dock_observation_reacquire_timeout |
| 160.660 | FAILED |
| 160.688 | 独立客户端 FAILED |

所有 t 为 trace 单调接收时间。全程真实 `LOCK_VERIFY` 进入 0 次，`LOCKED_FINAL_APPROACH` 0 次，`OBSERVATION_HOLD` 7 次；后者是失观保护，不应误记为锁定重试。无正向 Twist 指令，无 D 档。

## 对齐和观测

进入 1.05 m 范围后，横向 5 mm 与航向 0.5°门限只曾短暂同时满足。根据保留的 Action feedback 与不同时间戳观测近似重算，最长连续合格片段约 0.08～0.10 秒，未达到 0.5 秒。因此本轮没有反复触发停车锁定，但也没有完成稳定对齐。观测重建不是核心内部窗口的逐行导出，精确统计方法见 `analysis.json`。

最终 Action feedback 的横向误差约 +15.3 mm、航向误差约 +3.99°、纵向剩余约 0.301 m，对应距充电面约 0.761 m。最后 2 秒期间有有效观测间歇返回，但未持续满足重获所需的稳定窗口，最终由局部重获时限结束；不是已取消的整体或末段时限，也不是锁定后允许完整桩体离开视场的阶段。

本轮没有 `invalid_wheel_feedback` 或里程计失败。左右轮速差最大约 0.04827 m/s，超过原 0.10 m/s 门限的帧数为 0；因此不能声称本轮实车复现并通过了第十一次的差值触发条件。删除条件已由安装核对和软件回归验证。

## 停车及数据

结束后另行接收 3 秒新鲜反馈，301 条车辆状态、150 条 ECU 消息全部确认主速／轮速为零、N 档、motor=0、brake=true。随后关闭独立 launch、全部驱动与记录器，ROS 节点列表为空。

bag 正常关闭，共 127284 条消息，6651 帧雷达，覆盖启动至测试收尾；只在 recorder 退出后检查 SQLite，quick_check=ok。实时监控仅读取 trace 和日志，没有读取活动数据库。完整 Action feedback 已保存。

- `analysis.json`、`fault_window.json`：状态进入次数、反馈、对齐窗口与故障附近数据。
- `prestart_check.json`、`stop_confirmation.json`、`shutdown_confirmation.json`：启停检查与记录完整性。
- `motion_trace.jsonl`、`monitor.log`、`motion_summary.json`、`bag/`：原始记录。
- `relative_control_snapshot.py`、`feedback_odometry_snapshot.py`、`relative_docking.yaml`、`hardware.yaml`、`run_options.json`：本轮代码和配置快照。
