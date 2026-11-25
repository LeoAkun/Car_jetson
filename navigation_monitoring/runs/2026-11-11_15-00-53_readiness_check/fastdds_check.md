# FastDDS 只读残留检查

- 检查时间：2026-11-11T15:03:16.504864+08:00
- boot ID：`d53fafeb-91ac-4ceb-8254-8bb737042ac6`，与上午记录不同；uptime 9300.30 秒。
- /dev/shm：{'all_objects': 27, 'ports': 11, 'semaphores': 11, 'segments': 2, 'el_files': 3}。
- 可读 /proc 中未见相关 FD 或 mmap 占用；/proc/locks 未见与这些共享内存对象设备号及 inode 匹配的锁。
- 无权读取 FD 的进程：45 个；无权读取 maps 的进程：45 个。sudo 非交互只读探测提示需要密码，因此未获取全系统所有者覆盖。
- 判断：疑似孤儿残留；尚不足以完全排除不可读系统进程的持有，也不构成通信故障已证。
- 未见正在运行的导航/ROS/驱动进程；独立温度监控 PID 1446 仍在，未操作。
- /opt/ros/humble/bin/fastdds 存在；仅运行 --help 与 shm --help，确认 clean 子命令存在，未运行 clean。
- 未删除共享内存，未停止或启动进程，未运行 ROS、传感器或任务，未改参数/服务。

## 当前匹配进程

- PID 1446, UID 1001: `/usr/bin/python3 /home/akun/workspace/Car_jetson/thermal_testing/idle_crash_monitor.py --duration 86400 --interval 1 --label nav_field_test`

完整文件元数据、持有者扫描范围和原始 /proc/locks 见 fastdds_check.json。
