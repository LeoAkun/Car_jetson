# 2026-11-10 14:57:42 轮次故障定位与修复

## 结果

- 路线：3b2mus，113 → 106 → 107。
- 任务已接受，task_id 为 `task_1794293981`。
- 车辆全程未移动；终态车速为 0 km/h。
- 任务在进入有效导航前转为 `fault`，没有自动重试。
- 录包已正常封口，共 5 个 db3 分片，约 4.25 GB。

## 主故障

`lio_sam_imuPreintegration` 在 15:00:24.914967 以 exit code -6 退出，异常为：

```text
std::logic_error
basic_string::_M_construct null not valid
```

本轮并非 11 月 9 日出现过的 GTSAM `b0` 异常。该可执行文件同时承载
`IMUPreintegration` 和 `TransformFusion`，本次实际触发点在
`TransformFusion::imuOdometryHandler()`。

从 bag 中按接收顺序核对：

- 最新 `/lio_sam/mapping/odometry` 的 header 时间为 `1794294023.620991468`；
- 最后一条 `/odom_incremental` 的 header 时间为 `1794294023.533493280`，落后 87.498 ms；
- 原逻辑清理旧增量里程计时一次弹出 32 条，队列恰好变空；
- 随后仍访问 `front()/back()`，81 ms 后进程退出。

崩溃版反汇编也确认：队列首尾相等后仍复制无效 `Odometry`，复制字符串时进入
`basic_string::_M_construct null not valid` 的抛出分支。

## 已部署修复

在
`/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/src/imuPreintegration.cpp`
清理旧队列之后增加判空：队列为空时以 5 秒节流频率告警并返回，等待更新的
IMU 里程计，不再访问 `front()/back()`。

- `lio_sam` 单包构建成功；只有既有的 libtbb 版本链接警告。
- install 可执行文件 SHA-256：
  `a8ee0c1bb082ac053b263b47a5904db12e276d04c3fbdf120ad7d3a65a6280a3`
- install 可执行文件 Build ID：
  `df4c6d7a9f4d7ae01302ee6b0634cab0f345b3bf`
- `git diff --check` 通过。

此保护只修复本轮空队列崩溃。11 月 9 日 GTSAM `b0` 健壮性问题仍是独立未修项。

## 次生现象与 RViz 修复

LIO-SAM 退出后，Nav2 组件加载异常缓慢，60 秒健康检查缺少
`/follow_waypoints/_action/send_goal`，管理层因此上报启动超时。bag 显示 Nav2
容器之后仍继续初始化，所以该超时是本轮的次生现象，未证明 waypoint follower
插件本身损坏。为避免定位失效时仍继续等待并可能派单，`max_startup_time` 保持
60 秒，未上调。

RViz 同时因管理进程缺少桌面会话环境而报 xcb 无法连接显示。已在
`multi_map_navigation/process_manager.py` 中修复 Nav2 子进程环境：

- 保留当前完整环境；
- 仅从 `systemctl --user show-environment` 补齐缺失或空值的 `DISPLAY`、
  `WAYLAND_DISPLAY`、`XAUTHORITY`、`XDG_RUNTIME_DIR`、
  `DBUS_SESSION_BUS_ADDRESS`；
- 不覆盖已有非空值，不硬编码 Xauthority 临时路径；
- 查询失败时记录警告并沿用当前环境，不阻止 Nav2 启动。

目标测试结果为 `12 passed`，`py_compile` 和 `git diff --check` 均通过；使用安装
环境导入后，实测能解析当前桌面会话的 `DISPLAY=:0` 和对应 Xauthority。

## 收尾状态

- rosbag、保护监测器、cmd_vel→ECU 转换器均已停止。
- Nav2、LIO-SAM、RViz 及其残留进程均已清理。
- 修复前启动的管理进程组 `23785` 已在补丁完成后通过 SIGINT 干净退出，避免下轮复用旧代码。
- 传感器驱动保持运行，启动参数仍为 `start_cmdvel_2_ecu:=false`。
- 未远程切换驾驶模式、未主动制动、未回放录包、未自动派发第二次任务。

