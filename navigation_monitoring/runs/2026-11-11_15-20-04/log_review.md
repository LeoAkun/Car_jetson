# 本轮导航日志复核

复核文本截至：2026-11-11T15:36:18+08:00。三航点均有管理器 SUCCEEDED/到达证据，任务随后回到 idle；该结论只覆盖软件任务流程，不代替现场物理位置、人工接管和机械状态确认。仅只读文本日志与主代理保存的进程快照，未读取正在写入的 SQLite，未操作节点/进程/参数。

- 本轮目录：/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04。
- Nav2独立日志实际路径：/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_16-42-07/nav2_2026-11-11_15-20-32.log，管理器日志明确打印旧目录；本聊天没有移动文件或更改指针。
- dispatcher等待单次状态超时不能覆盖管理器已有接受和导航证据，任务没有重发。
- 15:21:38.453915750的ready DURABILITY不兼容已由主代理确认属于本轮新加detail_observer订阅：15:23:44.768停止旧观察器22028，改用volatile的v2观察器22278；未动生产。该警告不证明ready=false，bag的ready覆盖待关包实核。

## 管理器与导航阶段

- 15:20:32.323244930，management_stack.log:77：`[process_manager-1] [INFO] [1794381632.323244930] [process_manager]: LIO-SAM启动成功`
- 15:20:50.790907715，management_stack.log:118：`[process_manager-1] [INFO] [1794381650.790907715] [process_manager]: Navigation2启动成功`
- 15:20:50.795471430，management_stack.log:119：`[navigation_manager-4] [INFO] [1794381650.795471430] [navigation_manager]: Navigation2关键节点和生命周期状态均已就绪`
- 15:20:50.809466873，management_stack.log:121：`[navigation_manager-4] [INFO] [1794381650.809466873] [navigation_manager]: 向Navigation2发送目标: x=109.31, y=-46.35, yaw=-1.57`
- 15:20:50.812721780，management_stack.log:122：`[navigation_manager-4] [INFO] [1794381650.812721780] [navigation_manager]: 导航目标已接受`
- 15:21:52.098594640，management_stack.log:248：`[navigation_manager-4] [INFO] [1794381712.098594640] [navigation_manager]: 到达航点 1/3`
- 15:22:04.169351023，management_stack.log:277：`[navigation_manager-4] [INFO] [1794381724.169351023] [navigation_manager]: 向Navigation2发送目标: x=115.12, y=-143.71, yaw=-1.57`
- 15:22:04.175601458，management_stack.log:278：`[navigation_manager-4] [INFO] [1794381724.175601458] [navigation_manager]: 导航目标已接受`
- 15:26:02.099723000，management_stack.log:758：`[navigation_manager-4] [INFO] [1794381962.099723000] [navigation_manager]: 到达航点 2/3`
- 15:26:14.177915519，management_stack.log:788：`[navigation_manager-4] [INFO] [1794381974.177915519] [navigation_manager]: 向Navigation2发送目标: x=122.66, y=-249.50, yaw=-1.57`
- 15:26:14.186623934，management_stack.log:789：`[navigation_manager-4] [INFO] [1794381974.186623934] [navigation_manager]: 导航目标已接受`
- 15:29:58.610758988，management_stack.log:1242：`[navigation_manager-4] [INFO] [1794382198.610758988] [navigation_manager]: 到达航点 3/3`

## LIO预积分与ready

- 15:20:28.921891374，lio_sam_imuPreintegration_21488_1794381628868.log:1：`[WARN] [1794381628.921891374] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.`
- 15:21:38.453915750，lio_sam_imuPreintegration_21488_1794381628868.log:3：`[WARN] [1794381698.453915750] [lio_sam_imuPreintegration]: New subscription discovered on topic '/lio_sam/imu_preintegration_ready', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: DURABILITY_QOS_POLICY`
- 15:29:59.165063944，lio_sam_imuPreintegration_21488_1794381628868.log:4：`[INFO] [1794382199.165063944] [rclcpp]: signal_handler(SIGINT/SIGTERM)`
- 本观察器没有新建ready订阅。日志没有ready变化不等于false或持续true；不能凭文本复原Bool全部变化。

## 驱动与管理关键异常

- 15:20:07.686873697，real_drivers.log:166：`[ahrs_driver_node-5] [WARN] [1794381607.686873697] [ahrs_bringup]: check crc16 failed (ahrs), total_crc_errors=1`
- 15:35:52.809445659，real_drivers.log:867：`[ahrs_driver_node-5] [WARN] [1794382552.809445659] [ahrs_bringup]: Short IMU serial read for frame type: received=0 expected=1 total_short_reads=1`
- management：当前已读文本未匹配本次关注的CRC/短读/时间回退/gap/GTSAM/FastDDS锁等关键字；不等于全链路无异常。

## Nav2阶段及规划/控制证据

- 15:22:25.491438299，Nav2日志:137：`[component_container_isolated-1] [WARN] [1794381745.491438299] [planner_server]: GridBased: failed to create plan, invalid use: Starting point in lethal space! Cannot create feasible plan..`
- 15:23:17.971822589，Nav2日志:202：`[component_container_isolated-1] [WARN] [1794381797.971822589] [planner_server]: GridBased: failed to create plan, invalid use: Starting point in lethal space! Cannot create feasible plan..`
- 15:23:18.496304306，Nav2日志:206：`[component_container_isolated-1] [WARN] [1794381798.496304306] [planner_server]: GridBased: failed to create plan, invalid use: Starting point in lethal space! Cannot create feasible plan..`
- 15:23:18.561291021，Nav2日志:209：`[component_container_isolated-1] [ERROR] [1794381798.561291021] [bt_navigator_navigate_to_pose_rclcpp_node]: Failed to get result for follow_path in node halt!`
- 15:23:18.602491108，Nav2日志:211：`[component_container_isolated-1] [WARN] [1794381798.602491108] [bt_navigator_navigate_to_pose_rclcpp_node]: Node timed out while executing service call to global_costmap/clear_entirely_global_costmap.`

关键计数：`{"management_critical": 0, "driver_critical": 2, "nav2_critical": 5, "nav2_lethal_start": 3, "nav2_optimizer_mentions": 0, "nav2_control_rate_missed": 101, "nav2_zero_counter_warnings": 538}`。
控制循环未达期望 30Hz 共 101 条，说明存在调度延迟警告；仅凭这些日志不能量化全部控制周期。有效点数0共 538 条，沿用已知恒零计数缺陷边界，不能据此认定激光断流。本轮所读 Nav2 文本未匹配 Optimizer；无论有无该词，都不能单凭它证明旧 MPPI/GoalCritic 补丁部署状态。

## 最新监测快照和任务事件

```json
{
  "time": "2026-11-11T15:36:01.326+08:00",
  "event": "snapshot",
  "task_seen": true,
  "task_id": "task_1794381607",
  "robot_state": "idle",
  "robot_status": null,
  "processes": {
    "process_manager": [],
    "navigation_manager": [],
    "re_localization": [],
    "nav2_init_pose": [],
    "liosam": [],
    "nav2": [
      21671
    ]
  },
  "rates_hz": {
    "imu": 0.0,
    "scan": 0.0,
    "cmd_vel": 0.0,
    "odom": 0.0,
    "tf": 0.0
  },
  "freshness_sec": {
    "scan": 8.53,
    "vehicle_status": 8.51,
    "imu": 8.51,
    "tf": 361.61,
    "map->odom": 361.61,
    "odom": 362.29,
    "odom->base_link": 362.15,
    "cmd_vel": 362.1
  },
  "cmd_vel": [
    0.0587,
    0.0
  ],
  "vehicle": {
    "speed_kmh": 0.0,
    "steer_deg": 0.383,
    "drive_mode": 1,
    "is_autodrive": true
  },
  "goal_statuses": [
    4
  ],
  "cpu_percent": 0.6,
  "load": [
    1.74,
    2.99,
    3.12
  ],
  "memory_available_mb": 29433.8,
  "temperatures_c": {
    "acpitz": 27.8,
    "x86_pkg_temp": 35.0
  }
}
```
- `{"time": "2026-11-11T15:20:05.734+08:00", "event": "robot_state_changed", "state": "idle"}`
- `{"time": "2026-11-11T15:20:07.614+08:00", "event": "robot_state_changed", "state": "running"}`
- `{"time": "2026-11-11T15:20:50.811+08:00", "event": "goal_status_changed", "statuses": [2]}`
- `{"time": "2026-11-11T15:21:52.092+08:00", "event": "goal_status_changed", "statuses": [4]}`
- `{"time": "2026-11-11T15:22:04.171+08:00", "event": "goal_status_changed", "statuses": [2, 4]}`
- `{"time": "2026-11-11T15:26:02.093+08:00", "event": "goal_status_changed", "statuses": [4]}`
- `{"time": "2026-11-11T15:26:14.182+08:00", "event": "goal_status_changed", "statuses": [2, 4]}`
- `{"time": "2026-11-11T15:29:58.603+08:00", "event": "goal_status_changed", "statuses": [4]}`
- `{"time": "2026-11-11T15:30:00.819+08:00", "event": "robot_state_changed", "state": "idle"}`
- `{"time": "2026-11-11T15:35:53.318+08:00", "event": "terminal_candidate", "kind": "manager_lost"}`
- `{"time": "2026-11-11T15:36:01.332+08:00", "event": "monitor_stopped", "reason": "signal_2"}`

物理位置、人工接管及现场障碍以现场人员确认；文本日志不能单独给出唯一根因或车辆机械安全状态。

## 任务完成、恢复与清理边界

- 113、106、107 分别于 15:21:52.098594640、15:26:02.099723000、15:29:58.610758988 被管理器记录“到达航点”；最后一段紧前行 15:29:58.609633630 明确为 SUCCEEDED，15:29:58.611582350 开始关闭导航进程，15:30:00.819 监测到 idle。
- 三次 lethal-start 规划失败均在 106 段，伴随一次 follow_path halt result 错误和 clear_global_costmap 超时。此后仍到达 106、107，故日志支持任务从该阶段失败中继续完成；文本未提供足以唯一说明恢复机制和精确重规划恢复时刻的全部证据。
- 本次所读 management/LIO 文本未见 IMU gap、GTSAM、fatal 或 FastDDS 锁失败。该阴性结果不等于原始 IMU 全周期无异常，ready 的 Bool 历史仍须关包后审计；不把观察器未收到 ready 写成 ready=false。
- 15:20:07.686873697 AHRS CRC 错误是启动期实际警告，日志累计一次，不能据此唯一解释姿态/时间问题。15:35:52.809445659 的 Short IMU serial read 出现在 launch 已开始向驱动发送 SIGINT 的清理段；随后 15:35:52.915619203 驱动 signal_handler，PID 21155 finished cleanly。将其归入清理期串口短读证据，不冒充行驶期掉流或单独宣告其根因。
- 管理器 15:29:59.118504957 声称 navigation2 及其子进程已关闭，但主代理保存的 pre_cleanup_processes.json（15:34:47.665250+08:00）仍实录 PID 21671、PPID 1、PGID 21657 的 nav2_container。软件导航成功与子进程清理成功是两项独立结论：前者有到达/idle 证据，后者在该时刻未完成。
- 主代理说明自 15:35:52 开始 SIGINT 本轮 Nav2 残留及 management/drivers 并正常关闭 bag；本子任务没有自行检查实时进程或执行清理。最终清理是否无残留以主代理结束核对记录为准。
- nav2_init_pose 的 ExternalShutdownException/重复 rcl_shutdown 异常栈发生在 15:29:59.718 开始关闭该进程之后，随后 15:30:00.220450618 管理器报告关闭；它不是任务未到达的证据。
- monitor 15:35:53.318 manager_lost、15:36:01.332 signal_2 属于上述清理时段。末次快照的 cmd_vel=[0.0587,0] 已陈旧约 362 秒，不能解释为清理后仍持续发布的运动指令；其中 nav2 PID 只是一份当时的监测快照，不代替清理完成后的进程实核。
