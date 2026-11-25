# 允许倒挡正向滑行后的独立对接测试

2026-11-05，用户确认车辆已放回 P0 后授权启动一次实车测试。启动前确认自动驾驶 ON、轮速为零、空挡制动、只有一个 `/ecu` 发布者；后雷达以 `inverted=true` 连续 5 帧识别预期背板，推算突出充电面距离约 2.506 m。启动时未改动对接、硬件参数。

本次以 `dry_run:=false calibrated:=true auto_start:=false` 启动独立包，记录器先启动，再显式调用 `/standalone_docking/start` 一次；无自动重试。完整 rosbag 从硬件启动前持续记录到测试进程退出。`motion_trace.jsonl` 的 t 为记录器启动后的单调时间，不能与 bag 的 ROS 时间直接混用。记录器采用 `--manual-start --duration 180`，故摘要中的 `start_service_called=false` 表示启动由外部服务完成。

## 结果

未完成对接。首次停车原因为 `action_timeout`，动作发送后约 90 s 达到上限。超时前最后有效观测中，充电面位于 base_link 后方约 0.919 m，横向位置约 -0.0134 m。最终反馈确认主车速、左右轮速均为零，空挡，ECU motor=0、brake=true，自动驾驶仍为 ON。随后结束本次全部测试进程与记录器，ROS 节点列表为空。

| 记录器 t / s | 事件 |
|---:|---|
| 71.325 | 捕获人工 P0 并发送 Dock 动作 |
| 73.130 | APPROACH |
| 104.413 | FINAL_APPROACH |
| 122.332、130.767、147.607 | 目标短暂失效，进入 OBSERVATION_HOLD，停车重获后继续 |
| 148.875、149.015、154.575 | `forward_roll_in_reverse` 记录；不触发方向急停 |
| 154.245 | LOCK_VERIFY |
| 154.288 | `lock_alignment_lost_resume_tracking`，退回实时雷达控制 |
| 161.326 | STOPPING，`action_timeout` |
| 162.491 | FAILED，`action_timeout` |
| 162.492 | 首次 emergency_stop=true，发生于失败之后 |

## 正向前移与指令

停车前倒挡平均轮速超过 +0.01 m/s 共 3 段，约 70、70、130 ms，最大 +0.01334 m/s。倒挡期间所有正向平均轮速按采样作线性积分约 8.76 mm；另有空挡正向滑行积分约 2.52 mm。这些是轮速估计，未以外部测量独立确认车体位移。

`/cmd_vel_dock` 和 `/cmd_vel_selected` 的正向指令样本均为 0，范围为 -0.06～0 m/s。允许前移的修改已生效；本次失败不由正向保护触发。首次急停信号比首次超时停车晚约 1.17 s，是动作失败后的锁停结果。

## 尚未完成的环节

没有进入 `LOCKED_FINAL_APPROACH`，故本次未实车验证锁定后的里程计末段。LOCK_VERIFY 只维持约 43 ms，下一次观测即不满足对齐要求，未获得稳定锁定。

最后一段的倒车请求约 -0.025 m/s，ECU motor约0.09、R挡、释放制动，但记录器 t150～161 s 的实测平均轮速约 -0.00028 m/s，推进很慢；超时前仍距充电面约 0.92 m。应进一步核对该低速请求下底盘的实际运动响应，以及锁定对齐误差的波动。现有记录不能单独确定机械、底盘或低速编码原因，不宜将本次结果解释为锁定后盲退失败。

## 文件

- `bag/`：完整 ROS 记录，191193 条消息，包含 6202 帧完整后雷达扫描、TF、IMU、车辆反馈、ECU、命令、观测和状态。
- `motion_trace.jsonl`、`motion_summary.json`、`monitor.log`：运动与状态记录；JSONL 中雷达扫描抽稀，逐帧分析以 bag 为准。
- `launch.log`：启动及停止日志。
- `relative_docking.yaml`、`hardware.yaml`：本次参数快照。

本次未检查电池反馈，不表示已开始充电；没有修改控制参数或再次启动实车。
