# 传感器检查结束后的 FastDDS 只读复查

- 复查时间：2026-11-11T15:09:21.278558+08:00
- boot ID：`d53fafeb-91ac-4ceb-8254-8bb737042ac6`
- 对照基线：2026-11-11T15:03:16.504864+08:00，原 27 项。
- 当前数量与差异：{'all_objects': 64, 'baseline_objects': 27, 'new_names': 37, 'removed_names': 0, 'same_inode': 27, 'replaced_inode': 0}。
- 可见 FD 持有 0，mmap 持有 0，匹配 /proc/locks 锁 0。
- 原传感器 PGID 16624–16628 内仍可见进程：0。
- 温度监控 PID 1446 状态：S，未操作。
- 无权读取 FD/maps 的进程数量：43/43。
- 判断：未见活跃 FastDDS 占用，疑似孤儿对象；受系统进程读取权限限制，不作全系统绝对认定，也不能由文件存在断言通信故障。
- 未删除、停止或启动任何文件/进程；未运行 ROS 节点。

## 新增名称

- `fastrtps_07040c9590b0f371`
- `fastrtps_07040c9590b0f371_el`
- `fastrtps_411a9487fab923cd`
- `fastrtps_411a9487fab923cd_el`
- `fastrtps_54f923254cf4cd69`
- `fastrtps_54f923254cf4cd69_el`
- `fastrtps_c59005ea272fd94b`
- `fastrtps_c59005ea272fd94b_el`
- `fastrtps_fbc2848036f4fc74`
- `fastrtps_fbc2848036f4fc74_el`
- `fastrtps_port7413`
- `fastrtps_port7415`
- `fastrtps_port7417`
- `fastrtps_port7417_el`
- `fastrtps_port7419`
- `fastrtps_port7419_el`
- `fastrtps_port7421`
- `fastrtps_port7421_el`
- `fastrtps_port7423`
- `fastrtps_port7423_el`
- `fastrtps_port7425`
- `fastrtps_port7427`
- `fastrtps_port7429`
- `fastrtps_port7431`
- `fastrtps_port7431_el`
- `fastrtps_port7433`
- `sem.fastrtps_port7413_mutex`
- `sem.fastrtps_port7415_mutex`
- `sem.fastrtps_port7417_mutex`
- `sem.fastrtps_port7419_mutex`
- `sem.fastrtps_port7421_mutex`
- `sem.fastrtps_port7423_mutex`
- `sem.fastrtps_port7425_mutex`
- `sem.fastrtps_port7427_mutex`
- `sem.fastrtps_port7429_mutex`
- `sem.fastrtps_port7431_mutex`
- `sem.fastrtps_port7433_mutex`

## 已消失名称


## 当前相关进程

- PID 1446, PGID 1446, state S: `/usr/bin/python3 /home/akun/workspace/Car_jetson/thermal_testing/idle_crash_monitor.py --duration 86400 --interval 1 --label nav_field_test`
