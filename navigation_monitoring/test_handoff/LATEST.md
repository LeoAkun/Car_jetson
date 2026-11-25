# 最新实车测试交接

- 历史记录：`/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_16-45-24.md`。
- 运行目录：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-45-24`；报告：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-45-24/RESULT.md`。
- 地图/任务：3b2mus，110→114，task_1794386798。
- 开始16:45:24；管理器接受16:46:38；有效导航16:47:21起；110 ABORTED 16:48:59；约16:49:02 idle；首次非自动反馈16:49:18；用户16:50:41报告严重左偏并已接管；录包关闭16:50:41；清理16:51:32前（均2026-11-11 +08:00）。
- **有效导航、严重左偏、人工接管，完成0/2。** 110未成功到达，114未下发。用户未给物理接管精确时刻，不按消息到达顺序认定其发生在ABORTED之后。
- **主问题：全局定位/坐标基准冲突。** 用户说明车辆在110附近，软件却定位到距110约82.5米处，并规划约83米路径驶向110。软件轨迹大体跟随这条路径，现场严重左偏更符合全局路径与真实道路错位。
- 航点目录lat/lon和XY与当前3b2mus CSV不一致；显式起点会绕过GNSS距离检查，管理器也未校验当前map位姿到声明起点距离，因此82米冲突未阻止任务。
- 规划/控制：lethal-start 7次、优化器失败20次、Collision Ahead 2次、无进展1次、控制循环未达频率15次。自主速度峰值1.8km/h；转向反馈−25.94°至+24.26°。角度正负到实体左右未独立标定，不能仅凭符号断定转向反号。
- 接管边界：16:49:18首次`is_autodrive=false`并由mode1转mode3。最后10秒mode3/非自动、最高5.6km/h，是人工移动，已从自主段统计剔除。
- IMU：24,329条约99.853Hz；启动两条坏四元数和340/260ms跳变均早于ready与导航，随后未见同类问题。录包9分片、99,307条、约2.44GiB，正常关闭，SQLite和计数审计全部通过。
- 收尾：本轮所有PID退出，未见Nav2/LIO/re_localization/nav2_init_pose残留；thermal1446保留。未发布制动/零速/模式指令，未修改生产文件，未构建安装。
- 修复顺序：统一航点lat/lon与XY；显式起点也校验当前位置；发送首目标前核对map位姿与声明起点；加强重定位联合门槛；定位正确后再测转向/Ackermann/MPPI。
- 前序16:30无人工接管的ABORTED记录保留：`/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_16-30-04.md`；前序15:56严重右转入路沿后接管记录保留：`/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_15-56-01.md`，三轮不要混写。

更新：2026-11-11T17:06:58+08:00。仅测试、问题复核与记录，生产源码/配置/启动文件/安装程序/系统服务未改动。
