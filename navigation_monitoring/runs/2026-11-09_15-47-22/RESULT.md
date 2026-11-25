# 2026-11-09 15:47 同路线测试结束记录

用户要求保留上一轮参数、暂不修复偏左问题，再次发布3b2mus地图113→106→107任务；已确认车辆回113附近、路线清空且现场人员就位。未执行额外传感器预检，未修改导航、定位或驱动参数。原有独立status_reporter继续运行，本轮管理启动文件不重复创建它。

## 结果

- 15:48:25管理器接受task_1794210505，派单器退出0；接受任务不等于Nav2已开始行驶。
- 15:48:59重定位完成，随后启动LIO-SAM与Nav2。
- **15:49:01 IMU预积分进程PID23222因GTSAM IndeterminantLinearSystemException退出，exit -6，异常变量b0。**
- **15:50:01.732 Nav2启动健康检查超时**，controller_server响应超时，其余多个生命周期节点inactive，管理器终止任务。不能仅凭先后关系确定GTSAM异常与Nav2超时的完整因果，也不能仅凭model_dt警告认定Nav2卡死根因。
- 未收到NavigateToPose动作状态或反馈；cmd_vel_nav共0条，本轮没有完成有效航点行驶，不能评价106→107偏左是否复现。监控自动模式样本速度为0，人工模式后出现的速度不能归为自动导航超速。
- 用户明确“结束本次测试，我已经人工接管小车”。已停止本轮管理、驱动、监控和录包进程；Nav2残留容器PID23391在SIGINT/SIGTERM后仍未退出，随后清理。cleanup_verification.json确认本轮对应进程组无存活进程，bag metadata正常生成。
- boot ID始终c0655125-d5e3-4420-bd94-382edb4858a7，未观察到整机重启。未由本次操作发布底盘制动或模式切换消息。

## 保留数据与边界

录包在派单前开始，包含原始三维雷达3500帧、IMU39197条、底盘反馈32401条、原始任务图3条及TF/扫描/状态日志。/map存在多个消息类型，rosbag2未录取该同名话题；当前磁盘静态地图仍可独立引用。消息数以cleanup_verification.json和bag metadata为准。

保留management_stack.log、real_drivers.log、独立nav2日志、monitor.raw.jsonl、navigation_core_bag、配置哈希、operator_stop.json和cleanup_verification.json。本记录仅说明任务及停止状态，没有自动修复或重新发车。
