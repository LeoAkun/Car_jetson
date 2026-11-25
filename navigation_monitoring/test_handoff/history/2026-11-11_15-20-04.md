# 3b2mus 实车测试：113→106→107

本轮进入有效导航，三个航点均由 Nav2 返回 SUCCEEDED，管理器记录全部到达并回到 idle。用户明确确认“到达107后”才人工接管，用于“移动到新的地点准备做新的测试”，因此本轮按三个航点自主完成记录。路线完成不等于全部数据质量及控制问题已解决。

- 运行目录：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04`。
- 本历史记录：`/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_15-20-04.md`。
- 开始：2026-11-11 15:20:04.016+08:00；任务接受15:20:07.611–.614；任务完成15:29:58.610758988；idle15:30:00.819；录包关闭15:36:00.855979850；最终清理核对15:45:07.509644。
- 地图/路线：3b2mus，113→106→107；任务 `task_1794381607`。
- 用户指令：“直接开始启动导航，发布任务。”；随后“继续完成测试收尾，我已人工接管小车。”及上述接管澄清。
- 实车IMU PID21155运行ELF SHA256 `7ea6a0ee8e2d7986335b0b4fee18f59fbf3b8cab42fe016a41367806b630b321`，路径为当前Car_jetson安装程序，运行时证据见 [imu_runtime_identity.json](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/imu_runtime_identity.json)。
- boot ID `d53fafeb-91ac-4ceb-8254-8bb737042ac6` 本轮未变。与上午不同boot属于轮次背景，不推断重启原因。

## 任务和到点证据

| 阶段 | 北京时间 +08:00 | 证据 |
|---|---|---|
| bag/被动监测启动 | 15:20:04 | 先于实车驱动和派单 |
| 实车驱动、导航管理入口启动 | 15:20:05 | real_drivers PGID21042，management PGID21043 |
| 唯一一次派单/管理器接受 | 15:20:07.611534585 | StartEndGraph三次重复属于同一次工具派单；管理器仅一条WaypointList任务 |
| LIO ready=true 被bag收到 | 15:20:32.161781454 | Bool没有header，以bag接收时间记录 |
| LIO启动成功 | 15:20:32.323244930 | process_manager |
| Nav2启动成功/关键生命周期就绪 | 15:20:50.790907715/.795471430 | process_manager/navigation_manager |
| 113目标接受 | 15:20:50.812721780 | action UUID独立status=2 |
| 113到达 | 15:21:52.098594640 | action status=4及管理器到达1/3 |
| 106目标接受 | 15:22:04.175601458 | 第二action UUID status=2 |
| 106到达 | 15:26:02.099723000 | status=4及管理器到达2/3 |
| 107目标接受 | 15:26:14.186623934 | 第三action UUID status=2 |
| 107到达 | 15:29:58.610758988 | status=4及管理器到达3/3 |
| 任务回到idle | 15:30:00.819 | 独立monitor |

派单使用既有命令 `python3 navigation_tools/dispatch_navigation.py dispatch 107 --start 113 --map 3b2mus`，未使用force、未重发任务。派单工具未收到单次running状态，最终确认超时；管理器/独立monitor已经记录接受并执行，不能据工具超时归为派单失败。该确认时序值得代码聊天修复。

## IMU、定位与传感器

本轮真实驱动由既有 `ros2 launch launch_real run.launch.py` 启动，未额外重复前序传感器预检。前序传感器检查及DDS基线见 [readiness报告](/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_15-00-53_readiness_check.md)。GNSS在前序检查无fix，本轮已有946条有效话题消息，不能把前序无fix直接套用本轮。

关闭后bag审计：IMU94,521条，记录接收约100.008879Hz。header非递增0、非有限payload/covariance为0；有两条有限但异常四元数，均在启动15:20:07.683954017及.687789770，范数约2.908e-20及1.348e19。15:20:07.686873697另有AHRS CRC16错误。第二条异常header比前一条跳320ms，bag接收仅隔3.317ms；此后没有header>30ms间隔或异常四元数。不能把320ms header跳变解释为相同时长硬件断流。

IMU header持续超前bag接收，中位379.204ms，末条约393.662ms；bag到达时间不等同精确驱动/观察器主机callback时间。短测复核已经指出AHRS共享缓存CRC校验后提交、首条有效AHRS门槛、姿态有效性检查及imuTimestamp正向跳变/未来偏差的检查点，见 [源码只读复核](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-00-53_readiness_check/imu_source_review.md)。本轮未采原始串口字节，不将CRC和时间偏移认定为同一唯一根因。

ready bag仅一条true（15:20:32.161781454），无false记录；LIO及管理日志未见本轮大gap重置、恢复超时或GTSAM fatal，最终在任务完成后按管理器关闭。未出现false不构成连续始终ready的完整证明。本轮额外detail_observer初版错误请求TRANSIENT_LOCAL，15:21:38出现DURABILITY不兼容；主代理15:23:44停止该观察器并改为VOLATILE，仅修改本轮监控脚本，未改生产。其未收ready不能当作ready=false，bag先于LIO启动、已录到true。

## 规划与控制：任务成功，仍保留问题

106路段15:22:25.491、15:23:17.972、15:23:18.496三次 `Starting point in lethal space` 导致规划失败；15:23:18.561 follow_path halt结果失败，.602清global costmap服务超时。随后路线继续完成106和107。需要结合代价地图、足迹/膨胀、定位和重规划检查，单次不能唯一归因为障碍、地图、定位或控制。

Nav2日志另有101条控制循环未达到期望30Hz告警；不能仅凭警告计数当作每个实际控制周期。旧“有效点数0”计数告警538条沿用已知计数缺陷边界，不能据此断言激光断流。本轮bag记录489条plan、489条global costmap、7,120条local costmap、1,691条LIO mapping odom及10,400条cmd_vel。

全量VehicleStatus94,565条，反馈车速峰值2.5km/h、P99约2.1km/h；5秒monitor快照峰值仅2.1km/h，采用全量结果作为峰值结论。`/cmd_vel`最大正向linear.x=0.55555556m/s（2km/h），上游`/cmd_vel_nav`峰值0.56983012m/s（约2.051km/h）。速度>2km/h共5,339/94,565条（5.646%），最长连续样本段15:26:44.991628–15:26:46.451977，1.460350秒、该段峰值2.5km/h；按相邻超阈样本连续定义，不替代物理连续测速。底盘反馈速度并非独立地面真值；指令上限与实测速反馈超调须分开检查，不能据目标限速声称实车从未超过2km/h。

`/cmd_vel`558条负linear.x，最小−0.54368615m/s；只表示控制命令，不能单凭负值断言实际倒车。完整档位、转向及到点±10秒/每秒聚合窗口见离线审计。106到点15:26:02.099723时底盘仍反馈1.7km/h、转向−3.8475°；首零15:26:03.070617（晚0.970894秒），随后至下一目标接受15:26:14.186624保持1112条零速/N档/p_status=true，持续11.116007秒；下一目标接受后1.204045秒再次反馈非零。113和107判成功时分别反馈1.8/2.0km/h，首零分别晚0.711581/1.149873秒。目标成功与车辆已完全停稳须明确区分，106本轮有到点后的持续停稳证据。现场路沿距离、偏左程度、机械转向滞后、横墙效果及障碍真实距离没有独立外部测量，本轮不据软件到点直接判这些项目全部验收。

monitor最高CPU温度53℃、最低可用内存24,878.1MiB，未启用主动保护。测试聊天没有主动发布零速/制动或模式指令；既有节点自身控制及任务结束逻辑属于原程序行为。

## 人工接管与收尾

用户明确接管发生在107到达之后，用于转场准备下一次测试，非途中导航故障；精确接管时钟时间未提供。不依据drive_mode或历史current_run_dir推断现场位置/人工接管。

管理器15:29:59.118声称Nav2及子进程已关闭，但15:34:47实核发现PPID1的容器PID21671/PGID21657仍在。主代理15:35:52先SIGINT该本轮残留以及本轮管理/实车驱动；容器8秒未退出后SIGTERM，15:41:31复核仍活跃并实际持有4个DDS FD、3个mmap和匹配锁。15:42:32.720575再次核对PID/start_ticks/PGID/exe身份后仅对21671发SIGKILL，.820949已消失。最终15:45:07核对本轮所有进程组无成员，未见其他活跃ROS/导航/传感器候选。正常信号未退出问题及进程树漏清理均交代码聊天检查。

最终FastDDS对象119项，未删除；可见FD/mmap持有者和匹配SHM锁均为0。原导航残留持锁现场与后续释放现场分别保留；部分系统进程FD/maps不可读，所以“无可见占用”不替代全系统绝对证明。文件残留不等于通信故障，本轮所读日志未见open_and_lock_file类锁失败。独立温度监控PID1446保持原身份运行，未停止独立服务/上报器（基线未见上报器）。

15:35:52 IMU短读及nav2_init_pose关闭异常均发生在相应关停阶段，未写成行驶期故障；monitor清理期manager_lost也不改写任务已完成事实。

## 录包、证据与建议代码检查

bag正常关闭且有metadata，32分片、418,154条、14,334,070,784字节；覆盖15:20:05.734739975–15:35:52.800037672。32个SQLite quick_check全ok、逐topic/逐shard计数与metadata一致、反序列化错误0。本轮只在recorder退出后以只读immutable模式审计，无回放、未reindex、未打开活动SQLite；未再出现database locked。核心bag未包含完整原始速腾点云/深度图/地图图像，不能用它单独完整重放LIO输入。

Nav2独立日志实际仍写到旧目录 `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_16-42-07/nav2_2026-11-11_15-20-32.log`；副本已保存在本轮，未修改生产指针。事件±10秒文本窗口、完整原始文本与闭包保留。

- [run_state](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/run_state.json)、[monitor汇总](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/monitor_summary.json)、[日志复核](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/log_review.md)。
- [bag离线审计](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/offline_sensor_audit.md)、[完整数值与消息证据](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/offline_sensor_audit.json)、[录包元数据汇总](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/recording_summary.json)。
- [最终进程/DDS核对](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/final_cleanup_verification.md)、[此前真实持锁现场](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/cleanup_verification.json)、[强制退出身份和动作](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/nav2_orphan_final_termination.json)、[全部清理动作](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/cleanup_actions.json)。
- [本轮Nav2日志副本](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/nav2_2026-11-11_15-20-32.log)、[事件窗口索引](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/event_windows_index.json)、[文件索引](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/evidence_manifest.json)。

交代码聊天检查的具体文件/模块：

1. `/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp`及include/ahrs_driver.h：AHRS读取/CRC提交缓存、首条姿态有效性及imuTimestamp未来偏差/正向跳变。
2. `/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/process_manager.py`：Nav2子进程树/进程组回收、终止信号与存活确认，避免关闭成功误报；Nav2日志新轮目录。
3. `/home/akun/workspace/Car_jetson/navigation_tools/dispatch_navigation.py`：running订阅发现时序和确认超时，防止已接受任务被工具报未确认。
4. `/home/akun/workspace/Car_jetson/nav2/src/nav2/config/nav2_real_params.yaml`对应代价地图/足迹/规划参数和Nav2规划恢复调用、控制调度：106段lethal-start、清图服务超时、30Hz未达告警。仅建议检查，不在测试聊天修改。
5. `/home/akun/workspace/Car_jetson/real_ws/src/driver/car_control/cmdvel_2_ecu/cmdvel_2_ecu/convert.py`及chassis_driver：cmd_vel上限、负命令、底盘反馈速度超调、档位/制动时序；区分指令与物理实际运动。
6. `/home/akun/workspace/Car_jetson/nav2/src/nav2_init_pose/nav2_init_pose/nav2_init_pose.py`：关停上下文重复shutdown异常。

本聊天仅测试、停止本轮进程及写监控记录；未修改生产源码、配置、launch、安装程序或系统服务，未构建安装。最终补充到点/速度细节于2026-11-11T15:53:04.819757+08:00；整理时间：2026-11-11T15:49:39.607919+08:00。
