# 2026-11-03 独立倒车对接实车测试

用户已人工将车放回 `base_link` 距突出充电面 2.5 m 的 P0，并确认自动驾驶开关 ON。本次启动了独立功能包的一次实际对接测试，未完成对接。

启动命令：

```bash
source /opt/ros/humble/setup.bash
source /home/akun/workspace/Car_jetson/real_ws/install/setup.bash
ros2 launch charging_dock_standalone standalone_docking.launch.py dry_run:=false calibrated:=true auto_start:=true
```

本次日志时间为 Asia/Shanghai：

- 11:39:00 启动 Mux、相对对接控制器、速度转换器、反馈里程计、测试客户端、后雷达、雷达静态 TF、IMU 和底盘 CAN。
- 11:39:02.314 客户端记录 `Manual P0 captured; dry_run=False`。
- 11:39:48.753 返回 `FAILED dock_failed:dock_observation_invalid`，没有返回 `DOCKED`。
- 失败后读取 `/vehicle_status`，车速和左右轮速均为零、`shift_level=2`、`p_status=true`、`is_autodrive=true`。此反馈用于确认当时已停车，不代表运行中反馈一直为零。
- 11:41:34.936 发送 SIGINT 退出本次启动，所有子节点退出。部分 Python 节点因中断输出 traceback/非零退出码；对接失败发生在退出启动之前。

后雷达日志确认 `inverted:True`，传感器 `192.168.1.13`，主机 `192.168.1.140`，UDP 2369；CAN 使用 `br-sensors` 和 UDP 8002。IMU 成功打开固定 CP2102 串口，启动时出现一次 CRC 警告。没有证据将该警告与对接失败建立因果关系。

本次未运行扫描/里程计轨迹记录器，保留的是节点与启动日志。因此无法从本次记录计算实际行程、定位是哪一帧丢失目标，或判断是轮廓、关联门限、遮挡还是里程计误差导致观测失效。不能直接沿用 2026-10-30 测试的失败原因解释本次运行。

## 11:51 第二次记录

第二次测试在开始运动前启用了同步记录器，车辆从人工恢复的 P0 启动。结果、回放统计及限制见[run_02 记录](run_02/README.md)；原始 rosbag 风格 JSONL 为 `run_02/motion_trace.jsonl`。记录表明本次目标成功捕获并跟踪，车辆倒车至充电面约 0.69 m 后发生短时无效观测，控制器按当时实现立即失败停车。日志最后确认 ECU 零电机、制动、空挡及车辆速度/轮速为零。

这次实车测试暴露的是短时感知失效的恢复策略，不足以证明具体轮廓阈值导致失败。现在共享控制器已改为：有效闭环运动期间遇到无效或低质量目标即发零速并保持控制权；在 2.0 s 上限内重新稳定识别目标，且实测速度和角速度连续 0.5 s 低于停车阈值后才恢复；持续失观、反馈故障、里程异常或行程上限仍进入锁定停止。该修改只完成软件测试，尚未在车辆上重新验证。由于测试后车辆不在 P0，后续实车重测前必须人工移回 P0。

源码复核另发现速度单位约定冲突：`yunle_msgs/msg/VehicleStatus.msg` 将 `cur_speed` 标注为 km/h，底盘 `Msg.h`/`MsgBase.h` 将 CAN 反馈分辨率标注为 0.1 m/s，`Msg.cpp` 和底盘 README 也标为 m/s；现有对接消费者按 km/h 除以 3.6。已有静止与低速测试记录中的主车速为零，不能用于区分两种单位。该冲突尚未通过实车非零主车速、轮速与外部位移交叉标定解决，本次没有基于注释改变反馈换算或识别/控制门限。

再次运动测试前需人工重新确认车辆位于 P0，先启动数据记录再开始一次性对接，以同步保存扫描、目标观测、CAN 轮速、里程计和控制状态。失败不会自动重试。
