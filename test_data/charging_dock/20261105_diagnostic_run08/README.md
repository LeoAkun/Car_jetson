# 取消总时限后的独立自动对接实车测试

2026-11-05，用户确认车辆已放在指定 P0 后授权启动一次独立测试。安装配置与运行默认值均已确认：共享 `timeout=0`、独立客户端 `action_timeout=0`，锁定末段仍为 35 秒。使用 `dry_run:=false calibrated:=true auto_start:=false`，先启动完整 bag 和记录器，再启动硬件，确认停车、自动驾驶 ON、反馈新鲜及单个 ECU 发布者后调用一次 start 服务。记录器使用 `--manual-start --duration inf`，没有额外总期限，未自动重试或修改参数。

## 结果

**未完成对接，未完成桩位锁定。** 首次有效充电面距离约 2.506 m；倒车至约 1.05 m 后进入 `LOCK_VERIFY` 并正常请求空挡制动，但本地反馈里程计在制动滑行期间报告 `moving_without_direction_feedback`，发布紧停并停止更新 TF。随后控制器失去控制权而停车，最终因无法从冻结的运动估计确认停车报 `stop_not_confirmed`。

本次不是总时限停止，也未进入锁定后的里程计末段；动作启动约 55 秒即出现首次故障，故本次没有实车验证超过 90 秒继续运行的场景。最后有效观测距面 1.044 m 是反馈冻结前的测量，不能当作实际最终停距。

下列 t 为 `motion_trace.jsonl` 记录器的单调时间，不可直接与 bag 的 ROS 时间混用。

| t / s | 事件 |
|---:|---|
| 74.936 | 捕获人工 P0，发送 Dock Action |
| 76.181 | 首次有效观测，距面约 2.506 m |
| 76.731 | APPROACH，开始倒车 |
| 107.449 | FINAL_APPROACH |
| 126.801 | APPROACH，进入停车点前纠偏阶段 |
| 129.484 | LOCK_VERIFY，等待实测停车 |
| 129.503 | ECU motor=0、N 档、brake=true |
| 129.575 | 车辆主车速字段变成 0.2，N 档，有符号轮速仍为负 |
| 129.592 | 首次 moving_without_direction_feedback，随即首次 emergency_stop=true |
| 129.643 | STOPPING，control_not_owned |
| 132.623 | FAILED，stop_not_confirmed |

## 已定位的反馈判定问题

原始样本：t129.565 s，`cur_speed=0`、N 档，左右轮速约 -0.07367 / -0.04573 m/s，里程计仍有效。t129.575 s，`cur_speed=0.200000003`、N 档，左右轮速约 -0.08130 / -0.04827 m/s，平均约 -0.06478 m/s，自动驾驶与底盘制动反馈均为 true。

本次实车运行时，`FeedbackIntegrator.update()` 仅当 `cur_speed == 0` 时使用有符号轮速；主车速非零就切回依据挡位确定方向的路径。主车速非零且 N 档被判定为 `moving_without_direction_feedback`，即使此时新鲜、有效的有符号轮速已经提供倒退方向，也会锁存故障并停止发布里程计。这与本次制动换 N 档、车辆尚在滑行的时序不兼容。

因此，首次紧停来自反馈积分器的方向判定，不是雷达碰撞、桩体失观或锁定后的 35 秒超时。之后冻结的 TF/速度估计使控制器无法用持续新鲜的本地运动数据确认停车；`stop_not_confirmed` 是后续结果。该轮实车结束时仅定位并记录问题，没有修改实现或重启实车；后续修复见下节。

此前提到的 `cur_speed` 单位仍存在协议注释冲突；此处报告原字段值及当前代码的判断路径，不依据这一条样本宣称单位已完成标定。

## 后续软件修复与离线回放

启用有符号轮速时，积分器现持续使用轮速作为平移来源，主车速短暂非零不再改变方向来源；N 档滑行继续积分，实际停稳后才能锁定桩位。仍保留轮速过期、非有限值、左右轮速明显冲突及速度限制等检查。未启用轮速时没有方向来源的 N 档非零主速仍拒绝积分。正式运行的外部定位链不使用此积分器，共享控制器已经通过相同 N 档制动场景的停车等待验证。

`replay_feedback.py` 只读 bag，并按原里程计状态发布时刻近似重建定时器输入；不启动 ROS 节点、不发送指令。对同一组 CAN/IMU 原始时间戳分别运行原反馈来源条件和修复条件，比较原故障前后四秒。原条件在 ROS 时间 1793860154.7686362 再现 `moving_without_direction_feedback`；修复后该窗口没有反馈故障，位姿继续累计到轮速归零。该回放检验反馈积分行为，不等同于重新闭环实车对接或验证实际最终停距。

在已加载 ROS Humble 和当前 `real_ws/install/setup.bash` 的终端运行：

```bash
python3 /home/akun/workspace/Car_jetson/test_data/charging_dock/20261105_diagnostic_run08/replay_feedback.py
```

结果保存在 `feedback_replay_results.json`。此次后续修复仅做回放和隔离软件仿真，没有再次启动车辆。

## 结束确认

测试失败后另行接收 3 秒新鲜数据，获得 300 条车辆状态、150 条 ECU 消息，确认主速及左右轮速均为零、车辆与 ECU 均为 N、ECU motor=0、brake=true，自动驾驶仍 ON。随后结束独立包全部硬件与控制进程、记录器及 bag。进程检查与 ROS 节点列表确认无遗留测试节点，没有再次启动。

## 文件

- `bag/`：165517 条消息、5505 帧完整后雷达扫描，约 89.3 MiB，包含 TF、IMU、车辆反馈、ECU、两路命令、状态、观测、隐藏 Action feedback 及 `/rosout`。
- `motion_trace.jsonl`、`motion_summary.json`、`monitor.log`：运动与状态记录，JSONL 扫描抽稀，完整逐帧扫描应读取 bag。
- `fault_window.json`：首个异常前后的车辆、ECU、反馈状态、控制状态及紧停原始样本。
- `replay_feedback.py`、`feedback_replay_results.json`：修复前后的原始反馈离线回放与断言结果。
- `prestart_check.json`、`stop_confirmation.json`：启动前和结束时的新鲜反馈确认。
- `relative_docking.yaml`、`hardware.yaml`、`run_options.json`：配置及本次启动选项；记录器无限总时长通过命令行 `--duration inf` 实现。
- `launch.log`、`bag_record.log`：进程及记录日志。

本次未以电池反馈验收开始充电。
