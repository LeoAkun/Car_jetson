# 2026-11-10 16:42 IMU 重启后实车重试结果

- 路线：3b2mus，113 → 106 → 107
- 任务号：task_1794300179
- 操作：只终止现有 fdilink_ahrs 驱动子进程，由原 launch 的 respawn 机制重新拉起；未重启其他传感器
- 结果：任务进入 running，但 LIO-SAM/Nav2 启动阶段故障，未进入有效行驶
- 车辆状态：全程 0 km/h，cmd_vel 为 0

## 根故障

2026-11-10 16:44:02 +08:00，lio_sam_imuPreintegration 的健壮性检查在优化 IMU 队列发现时间间隔异常：

- stage=integrate_optimization_imu
- key=1
- imu_time=1794300241.747379
- previous_time=1794300241.616163
- dt=0.131216 s
- current limit=0.1 s

进程记录结构化 FATAL 后主动退出，LIO-SAM/Nav2 联锁终止任务。本次与上一轮的实时队列 dt=0.359815 s 属于同类时间间隔异常，但发生在优化队列；重启 IMU 未消除问题。
