# 最终清理核对

- 核对时间：2026-11-11T17:01:40+08:00；boot ID `d53fafeb-91ac-4ceb-8254-8bb737042ac6`。
- 本轮bag、monitor、detail observer、dispatch、management、real drivers PID均已退出；未见Nav2、LIO、re_localization或nav2_init_pose残留。
- 16:50:41首次SIGINT；实车launch重拉两个传感器子进程，16:51:15再次SIGINT仍重拉；核对PID29405/PGID29405/start_ticks1549924后，16:51:32发SIGTERM，进程组退出。
- FastDDS对象文件125项，未删除；`lsof`未见这些对象的可见持有者。文件存在不等于DDS故障；本次未把全局 `/proc/locks` 条数误写为DDS锁。
- 独立thermal PID1446保持运行，未操作。未发布车辆制动、零速或模式指令。
