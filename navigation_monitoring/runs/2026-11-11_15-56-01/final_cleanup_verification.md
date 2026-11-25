# 15:56:01轮最终清理核对

- 时间：2026-11-11T16:06:23.687183+08:00。boot ID与本轮开始一致：True，`d53fafeb-91ac-4ceb-8254-8bb737042ac6`。
- 16:03:48发现的Nav2 cloud残留PID24214/PGID24195保留在cleanup_verification.json/.md。该记录含初始发现和16:05:15后续扫描，未覆盖。
- 主代理16:04:47.195513验证PID24214/start1249753/PGID24195/exe后单PID SIGINT，16:04:47.295989确认退出，证据nav2_cloud_residual_termination.json；本观察员未发信号。
- 当前核查组[23561, 23562, 23563, 23598, 23599, 23892, 24195]，剩余成员0；/proc/24214存在=False，/proc/24195存在=False。
- 除thermal外活跃ROS/导航/传感器/helper/ROS2 daemon候选=0。
- thermal PID1446状态S，start_ticks5651和命令与pre_cleanup相同=True，保留未动。
- FastDDS对象{'objects': 125, 'ports': 49, 'semaphores': 49, 'segments': 11, 'el_files': 16}；可见FD/mmap持有=0/0，匹配/proc/locks锁=0；未删除文件。
- FD/maps不可读进程数0/0，所有者核查有权限范围限制；文件存在不等于DDS故障。
- metadata.yaml存在=True；本次未读写SQLite数据库。
- 用户最新现场澄清（由主代理转述）：启动后不久严重向右转、进入路沿及其所述全局地图障碍区，然后人工接管。精确接管时间未提供；不能由消息到达或记录顺序推定其与15:59:43软件ABORTED的实际先后。软件ABORTED及自动关栈事实保留，现场接管原因不再记为未知。
- 本次仅只读核查并写监测记录，未启动ROS/节点、清理DDS文件、操作车辆或发任何进程信号。
