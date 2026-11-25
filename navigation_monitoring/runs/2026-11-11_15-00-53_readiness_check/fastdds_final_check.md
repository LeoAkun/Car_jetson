# 传感器补测结束后的最终 FastDDS 只读复查

- 复查时间：2026-11-11T15:14:37.872035+08:00
- boot ID：`d53fafeb-91ac-4ceb-8254-8bb737042ac6`
- 当前对象：{'all_objects': 64, 'ports': 22, 'semaphores': 22, 'segments': 7, 'el_files': 13}。
- 相对 original 基线 27 项：新增 37，消失 0，原inode保留 27，同名替换inode 0。
- 相对 after_sensor 基线 64 项：新增 0，消失 0，原inode保留 64，同名替换inode 0。
- 可见 FD/mmap 持有者：0/0；匹配 /proc/locks 锁：0。
- 已停止 PGID 16624–16628、17588 内可见剩余成员：0。
- thermal PID 1446：状态 S，命令与前次一致，未操作。
- FD/maps 不可读进程数：47/47；不能完全排除不可读系统进程持有。
- 判断：可见范围未见活跃 ROS/导航/传感器残留或 FastDDS 持有者；共享内存为疑似孤儿残留，文件存在不等于通信故障。
- 本次未删除共享内存、未停止或启动进程、未运行 ROS，前两次证据保留。

## 当前相关进程

- PID 1446, PGID 1446, state S: `/usr/bin/python3 /home/akun/workspace/Car_jetson/thermal_testing/idle_crash_monitor.py --duration 86400 --interval 1 --label nav_field_test`

## 相对中间64项新增对象

