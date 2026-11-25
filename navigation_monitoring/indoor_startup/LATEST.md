# 最新室内无任务启动测试

北京时间 2026-11-11 14:20:08～14:23:54，14:24:03 完成清理。

[本轮报告](/home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/2026-11-11_14-20-08/RESULT.md) · [IMU 分析](/home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/2026-11-11_14-20-08/imu_timing_analysis.md)。前两轮报告保留在 `2026-11-11_13-08-19/RESULT.md` 和 `2026-11-11_13-24-18/RESULT.md`。

真实 IMU、真实雷达、LIO-SAM 与 Nav2；人工地图及 map→odom，无 GPS、无任务、无底盘控制。8 节点 Active 后保持 180 秒，LIO ready 未回落，测试进程已全部退出。

22,343 条 IMU、100.022 Hz，最大接收间隔 22.844 ms，无 >200 ms 接收中断。但启动期再次出现 330 ms header 跳变、两条无效四元数和一次 AHRS CRC16 错误；header 持续超前接收主机，中位数 365.746 ms。启动检查通过，IMU 数据质量不能判为全部正常。

用户已撤回上一轮“全程未移动”的说法，改为可能移动；不根据位置变化判定静止漂移。本轮以 IMU 和启动连续性观察为主。

未修改生产源码、参数、已安装程序或系统服务。此测试不等同于运动定位、生产大地图/RViz 负载或任务流程验收。
