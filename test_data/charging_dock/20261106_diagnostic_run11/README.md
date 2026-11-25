# 锁定抖动修复后的第十一次独立实车测试

2026-11-06，用户将车放回指定 P0 后授权一次测试。根代理是唯一硬件控制者；先启动 trace 与 rosbag 记录，再启动独立包，保持 `dry_run=false`、`calibrated=true`、`auto_start=false`。4 秒静态检查通过后仅调用一次 `/standalone_docking/start`。没有自动重试，没有修改运行参数或生产代码。

## 结果

**本轮未完成对接，距充电面约 1.08 m 时因左右轮速反馈异常自动停车；尚未进入 LOCK_VERIFY，不能据此确认锁定抖动修复的实车效果。**

安装核心与源代码一致：停车前连续 0.5 秒、至少 3 帧对齐，默认 5 mm／0.5°门限；验证失败后至少经过 1 秒且净位移达到 1 cm 才可重新积累对齐窗口。整体及锁定末段时限均为 0。

静态检查确认自动驾驶 ON、主速和双轮速为零、N 档制动、里程计有效、单一 ECU 发布者、CAN／IMU／雷达／里程计新鲜。115 帧有效静态目标，最后一帧充电面距离约 2.503 m。

| trace t / s | 事件 |
|---:|---|
| 90.002 | 请求启动，WAIT_SENSORS |
| 90.552 | DOCKING |
| 92.342 | APPROACH，开始倒车 |
| 124.793 | FINAL_APPROACH |
| 131.708 起 | 多次短暂 OBSERVATION_HOLD，停车后恢复；这些不是锁定验证重试 |
| 190.254215 | 车辆反馈左轮 +0.0127025、右轮 +0.3175627 m/s，R 档，主速字段 raw=0.3000000 |
| 190.254577 | 反馈里程计报 invalid_wheel_feedback 并锁存失败 |
| 190.254702 | 首个 emergency_stop=true |
| 190.257027 | ECU 已切 N 档、motor=0、brake=true |
| 190.311 | STOPPING reason=control_not_owned |
| 190.851 | FAILED reason=control_not_owned |
| 190.902 | 独立客户端 FAILED |

上述 t 为 trace 单调接收时间；毫秒级差值表示记录的接收顺序，不当作硬件执行延迟。全程未见正向速度命令或 D 档命令。独立反馈积分器允许此前授权的短暂正轮速，本轮停车由左右轮速一致性校验触发。

## 停车原因

`FeedbackIntegrator.update` 的 `invalid_wheel_feedback` 检查包含非有限值、任一轮绝对速度超过 0.5 m/s、左右轮速差超过 0.10 m/s。本帧两轮均有限，绝对值均小于 0.5 m/s，但差值约 0.30486 m/s，明确超过第三项门限。因此并非反馈过期、N 档方向未知或 35 秒／90 秒时限。

该异常反馈在后续一条车辆消息中重复，约 20 ms 后右轮恢复为 -0.005081 m/s。现有记录能确认异常值与停车链，不能确认是实际轮滑、底盘上报还是解码通信问题；本轮没有采集原始 CAN 帧，不能继续归因。`control_not_owned` 是紧停后控制权撤回的后继状态，不是最早异常。完整证据见 `analysis.json`、`fault_window.json`。

## 收尾与数据

测试结束另行接收 3 秒新鲜数据：301 条车辆状态和 150 条 ECU 消息全部确认主速／双轮速为零、N 档、motor=0、brake=true。随后关闭 launch、所有硬件与记录器；ROS 节点列表为空，没有测试进程残留。

本轮 bag 正常关闭，126620 条消息、6795 帧雷达，覆盖硬件启动到测试收尾，SQLite quick_check=ok。仅在 recorder 退出后检查数据库，实时监控只读 trace 和日志。新增完整 Action feedback 至 trace 与 bag，保留控制器精确误差。时间、停止确认与完整性见 `shutdown_confirmation.json`。

- `relative_docking.yaml`、`hardware.yaml`、`relative_control_snapshot.py`、`run_options.json`：本轮配置与控制核心快照。
- `prestart_check.json`、`stop_confirmation.json`、`shutdown_confirmation.json`：启停及完整性确认。
- `motion_trace.jsonl`、`monitor.log`、`motion_summary.json`：实时记录与终态。
- `bag/`：完整频率传感器、控制、Action feedback 与 TF。
