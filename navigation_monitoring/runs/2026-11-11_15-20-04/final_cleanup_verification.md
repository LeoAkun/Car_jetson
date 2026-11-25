# 本轮最终清理后的只读核对

- 核对时间：2026-11-11T15:45:07.509644+08:00
- boot ID：`d53fafeb-91ac-4ceb-8254-8bb737042ac6`；与本轮开始一致：True。
- 原15:41:31活跃Nav2持锁现场保留：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04/cleanup_verification.json`，未覆盖。
- 主代理已在15:42:32.720575校验身份后向PID21671发送SIGKILL；具体动作/退出证据见nav2_orphan_final_termination.json与cleanup_actions.json。本观察员未发信号。
- 核对PGID：[21016, 21017, 21042, 21043, 21337, 21657, 22028, 22278]；剩余可见成员：0；/proc/21671存在：False；/proc/21657存在：False。
- 除独立温度监控外活跃ROS/nav/sensor候选：0。
- thermal PID1446：状态S；start_ticks与命令和先前一致：True，保留未动。
- FastDDS对象：{'all_objects': 119, 'ports': 49, 'semaphores': 49, 'segments': 9, 'el_files': 12}；相对活跃孤儿现场119项，新增0、消失0、同inode保留119、同名替换0。未删除任何对象。
- 可见相关FD/mmap持有者：0/0；匹配SHM锁：0；numeric owner21671/21657的锁：0。
- FD/maps不可读系统进程数：43/43，归属覆盖存在权限局限。未见占用的残留对象只能称疑似孤儿，不等于已证明DDS通信异常。
- 用户澄清：在107到达后为转场准备下一次测试而人工接管，非途中故障；实际接管时钟时间未提供。此条不改写原始目标结果或持锁现场。
- 本核对仅写本轮监测证据；未运行ROS、启动节点、删除文件、发送进程信号或操作车辆。
