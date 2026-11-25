# 第十三次独立自动充电实车测试

2026-11-10，用户将车辆放在 P0 后授权单次测试。先启动 trace、rosbag 和独立功能包（`dry_run=false`、`calibrated=true`、`auto_start=false`）。第一次预检自动驾驶为 OFF，因此没有启动运动；用户确认切到 ON 后，重新进行 4 秒静态检查，全部通过，随后仅调用一次 `/standalone_docking/start`。未自动重试，也未修改生产代码或参数。

## 结果

**本次未完成充电对接。在距充电面约 0.79 m、尚未进入锁定验证时，因目标未能在 2 秒内持续稳定重获而停车。** 软件结果为 `FAILED dock_observation_reacquire_timeout`。

重做的预检收到 403 条车辆状态、201 条 ECU、401 条 IMU 和 118 帧后雷达；速度为零、自动驾驶 ON、N 档制动、里程计有效、ECU 发布者唯一，全部数据新鲜。114 帧有效静态目标，最后充电面距离 2.5024 m。

| trace t / s | 事件 |
|---:|---|
| 527.437 | WAIT_SENSORS，单次启动 |
| 527.987 | DOCKING |
| 529.785 | APPROACH |
| 560.943 | FINAL_APPROACH |
| 579.062 | 首次 OBSERVATION_HOLD，随后重获恢复 |
| 580.595～580.915 | 短暂 forward_roll_in_reverse 反馈，未触发故障 |
| 609.860 | 距面约 0.8 m，失观停车，重获后继续 |
| 643.023 | 最后一次 OBSERVATION_HOLD |
| 645.023 | STOPPING，dock_observation_reacquire_timeout |
| 645.577 | FAILED |
| 645.587 | 独立客户端 FAILED |

时间为 trace 单调接收时间；运动测试从 DOCKING 至客户端终态约 117.60 秒。记录器在启动前等待人工打开开关，因此记录总长明显大于运动时长。

## 监控结论

真实 `LOCK_VERIFY` 和 `LOCKED_FINAL_APPROACH` 均进入 0 次，`OBSERVATION_HOLD` 进入 6 次。看到的停车与恢复来自失观及重获流程。本次尚未执行锁定位姿后不依赖完整桩体的末段倒车。

最后 Action feedback 横向误差约 +18.37 mm、角度误差约 +3.207°，不满足锁定要求的 5 mm、0.5°以及连续 0.5 秒稳定窗口。最后有效观测仍有返回（距面 0.79347 m），但间歇有效不等于满足持续稳定重获条件。整个动作共有 14 条 `no_complete_face_in_gate` 和 45 条 `warming_up`；单凭这些原因不能确定究竟是扫描边缘、轮廓变化还是关联门限导致每次失观。

依据不同时间戳观测与相邻 Action feedback 近似重算，进入 1.05 m 以内后有 1327 帧有效观测，其中 17 帧同时满足横向及角度门限，最长连续跨度约 0.135 秒（5 帧），未达到 0.5 秒。t≈605.53 s 时距面约 0.891 m，横向约 1.32 mm、角误差 -0.184°，曾接近锁定，但没有持续保持。该重算不是控制核心内部窗口逐行导出，详细方法和统计见 `analysis.json`。

无正向 Twist 指令、无 D 档；存在短暂正轮速反馈，按当前已允许的逻辑正常积分。最大左右轮速绝对值分别约 0.0648、0.0749 m/s，最大左右差约 0.0546 m/s；未发生轮速有效性故障。原双轮差值门限已经删除，本次没有复现超过原 0.10 m/s 的样本。

此次没有触发被禁用的整体或锁定末段时限。首个 emergency_stop 在 FAILED 之后约 1 ms 发布，是失败收尾，不是最先触发停车的原因。

## 停车和记录

trace 终止时仍有很小的轮速反馈，所以额外订阅了 3 秒新鲜状态后才退出硬件：301 条车辆状态、150 条 ECU 均确认主车速及双轮速为零、N 档、motor=0、brake=true。然后向本轮唯一 launch 进程发 SIGINT，确认结束后关闭 rosbag；ROS 节点列表为空，测试控制及传感器进程已退出。部分 Python 节点在 SIGINT 收尾时打印 KeyboardInterrupt 或重复 rcl_shutdown 异常，发生在停车确认之后，不是本次运动失败原因。

bag 正常关闭，365536 条消息、19995 帧后雷达，SQLite `quick_check=ok`。只在 bag 退出后读取数据库；实时监控仅读取 trace、日志和 ROS 反馈。

- `prestart_check_autodrive_off.json`：初次开关 OFF 的检查。
- `prestart_check.json`、`start_service.log`：用户切到 ON 后的通过检查和单次启动响应。
- `analysis.json`、`fault_window.json`：状态、速度和故障附近原始数据。
- `stop_confirmation.json`、`shutdown_confirmation.json`、`bag_integrity.json`：停车、退出和数据完整性证据。
- `motion_trace.jsonl`、`monitor.log`、`motion_summary.json`、`bag/`：原始记录。记录器使用 manual-start，summary 内 `start_service_called=false` 仅指记录器自身没有调用；实际调用记录在 `start_service.log`。
- `relative_control_snapshot.py`、`feedback_odometry_snapshot.py`、`relative_docking.yaml`、`hardware.yaml`、`run_options.json`：本轮使用的代码和参数快照。
