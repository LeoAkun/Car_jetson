# 2026-11-10 16:36 实车测试结果

- 路线：3b2mus，113 → 106 → 107
- 任务号：task_1794299847
- 任务进入 running：2026-11-10 16:37:27 +08:00
- 结果：导航栈启动阶段故障，未进入有效行驶
- 车辆状态：全程 0 km/h，cmd_vel 为 0；未远程制动或切换驾驶模式

## 首个根故障

2026-11-10 16:38:24 +08:00，lio_sam_imuPreintegration 的健壮性检查发现实时 IMU 时间间隔异常：

- stage=integrate_realtime_imu
- key=20
- invalid realtime IMU dt=0.359815 s
- last_imu_realtime=1794299903.819076538
- current IMU time=1794299904.178891

进程按设计记录结构化 FATAL 并以 exit code 1 退出。进程管理联锁随后终止 LIO-SAM/Nav2，任务转为 fault，未自动重试。

## 数据留存

- rosbag 已正常关闭并生成 metadata.yaml
- db3 共三段，约 2.21 GB
- 管理栈、cmdvel_2_ecu、本轮监测器和录包均已停止
- 原传感器启动进程保持运行
