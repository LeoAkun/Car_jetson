# 本轮结束后的只读清理核对

- 核对时间：2026-11-11T15:41:31.882699+08:00
- 运行目录：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-20-04`
- 当前boot ID：`d53fafeb-91ac-4ceb-8254-8bb737042ac6`；与本轮开始相同：True。
- 核对进程组：[21016, 21017, 21042, 21043, 21337, 21657, 22028, 22278]。
- **核对时清理尚未完成：Nav2孤儿PID21671、PGID21657仍在，state=S、PPID1、start_ticks1033757。** cleanup_actions.json已记先SIGINT后SIGTERM，本次观察未再发信号。
- 其余指定组21016/21017/21042/21043/22028/22278和dispatch21337未见成员。
- PID1446温度监控：状态S，与派单前命令相同：True；本次未操作。
- FastDDS对象：{'all_objects': 119, 'ports': 49, 'semaphores': 49, 'segments': 9, 'el_files': 12}；相对派单前64项，新增62、消失7、同inode保留52、同名替换inode5。
- PID21671持有可见FastDDS FD 4项、mmap 3项；/proc/locks匹配锁4项。存在明确活跃持有者，不能将全部共享内存归为孤儿。
- /proc/locks中部分数字owner为21657，但/proc/21657不可见，不能据此单独断言21657仍存活；可能为继承/开放文件描述锁留下的创建者标识。
- FD/maps不可读系统进程：43/43；其他对象归属覆盖有限，文件存在不构成通信故障已证。
- 用户已报告人工接管；实际接管时刻和原因未提供。本次仅核对收尾状态，不作车辆控制。
- 本核对没有发送信号、清理DDS文件、启动ROS/传感器/导航或修改参数/系统服务。

## 剩余导航进程

- PID21671, PGID21657, state=S: `/opt/ros/humble/lib/rclcpp_components/component_container_isolated --ros-args --log-level warn --ros-args -r __node:=nav2_container --params-file /tmp/launch_params_rd2fz9yl --params-file /tmp/launch_params_onwbt8c6 -r /tf:=tf -r /tf_static:=tf_static`

完整共享内存对象列表、可见持有者、原始/proc/locks和进程元数据见cleanup_verification.json。
