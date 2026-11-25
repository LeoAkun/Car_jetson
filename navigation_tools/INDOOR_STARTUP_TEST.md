# 室内真实 IMU 与 Nav2 静止启动测试

在未建立现场地图、GPS 无信号时，使用真实 IMU、真实速腾三维雷达，启动四个真实 LIO-SAM 节点及组合模式 Nav2。程序不发送导航目标、不接入 MQTT，不启动底盘、CAN、速度转换、车门或报警节点。

```bash
bash /home/akun/workspace/Car_jetson/navigation_tools/run_indoor_startup_test.sh \
  --baseline 20 --lio-timeout 90 --nav2-timeout 180 --hold 120
```

脚本先检查设备是否已被其他传感器/导航进程占用；有冲突时退出并保留原进程。此入口自行启动真实驱动，测试结束仅清理本次进程组。默认打开 IMU 串口时沿用真实驱动的 reset_on_open 设置。

测试补充的输入只有 40 米方形空白地图及单位 `map→odom`；`odom→base_link`、里程计和预积分 ready 必须来自真实 LIO-SAM。雷达及 IMU 安装 TF 沿用现有整车外参。三维点云投影到 `/laser/scan_merge_laser` 供代价地图使用，未测试原前后单线雷达及其融合链路。

全部节点使用真实时间。DDS 限定本机、域 88；Nav2 速度输出改为 `/indoor_startup/cmd_vel_nav`、`/indoor_startup/cmd_vel`。测试期间无任何 IMU 模拟或回放发布者，也没有模拟里程计发布者。

Nav2 使用生产参数副本、实际 MPPI/Smac/行为树插件、组合容器及两个生命周期管理器。临时配置只替换地图/行为树 XML 路径。LIO-SAM 参数副本关闭地图导出及回环；没有启动 GPS 转换或基于历史地图的重定位。

运行结果保存在 `/home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/<北京时间>/`：

- `events.jsonl`：启动阶段、进程及退出事件。
- `observations.jsonl`：每秒各阶段统计、TF、生命周期状态、隔离速度与 action 状态。
- `imu_samples.jsonl`：真实 IMU 原始数值、消息时间戳和接收时刻，便于离线对比。没有录制或读取 SQLite bag。
- `imu.log`、`lio_*.log`、`nav2.log`：真实节点日志。
- `result.json`：启动、最终数据新鲜度及无任务/速度输出检查的结果。`passed` 表示这些启动检查通过，不等于 IMU 时序、定位精度或整车导航验收通过；需同时判读原始 IMU、CRC、ready 变化和逐秒状态。

本测试不覆盖生产任务管理器的服务超时、GPS/重定位、真实大地图及 RViz 带来的完整负载，也不验证车辆行驶、转向、制动或动态定位精度。

最新一次记录及前轮索引见 [LATEST.md](/home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/LATEST.md)。

已结束采集可离线分析，不连接硬件或 ROS：

```bash
python3 /home/akun/workspace/Car_jetson/navigation_tools/analyze_indoor_imu.py \
  /home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/2026-11-11_13-24-18 \
  --compare /home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/2026-11-11_13-08-19
```

会在主采集目录生成 `imu_timing_analysis.json`、`imu_timing_analysis.md` 及 PNG/SVG 时间图；原始采集和对比轮目录保持不变。输入必须已有 `result.json` 和 `test_finished` 结束事件。
