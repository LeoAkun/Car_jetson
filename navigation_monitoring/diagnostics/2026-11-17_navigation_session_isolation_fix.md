# 连续导航冲突与重定位失败修复说明

日期：2026-11-17

## 结论

连续任务的主要风险不是“重定位算法偶尔不稳定”，而是上一轮导航子栈、定位点云和 `map -> odom` 可能被下一轮复用。修复后的启动条件改为：旧进程组已验证退出、本轮会话身份一致、本轮点云时间有效、本轮定位结果与持续刷新的 TF 一致、路线地图与实际栈地图一致。任一证据缺失都拒绝发 Nav2 目标。

## 已实现的顺序

1. **进程所有权与清理屏障**
   - 同名存活进程不能被新 `Popen` 句柄覆盖。
   - 保存每个启动项的 PGID，关闭时检查整个进程组，执行 `SIGTERM -> SIGKILL`，父进程退出但子进程仍活着不再算关闭成功。
   - 完成任务、故障恢复和地图切换都按 `navigation2 -> liosam -> nav2_init_pose -> re_localization` 关闭，并连续两次确认全停止。
   - 状态未知、响应缺字段、部分运行均 fail-closed。
   - 增加 `process_manager` 文件锁；第二个管理器无法取得所有权时，整套管理 launch 随即退出。
   - 启动前扫描未纳管的定位、LIO-SAM 和 Nav2 残留；发现 PID 后拒绝叠加新实例。状态查询也会把这些 PID 报为故障，而不是误报全部停止。

2. **定位会话与点云隔离**
   - 每次 `re_localization` 启动生成新的 UUID、地图名和 ROS 时钟起始时间。
   - 服务请求、服务响应和最终定位结果均携带这三个字段。
   - 重定位服务只缓存本进程、本会话开始之后收到的第一帧点云；旧会话、旧地图或旧时间的请求和点云会被拒绝。
   - `StartProcess` 响应把本轮会话身份返回给调用端，调用端只等待这个明确的 UUID。

3. **唯一 `map -> odom` 发布者**
   - `nav2_init_pose` 不再直接广播 TF，只发布 `LocalizationResult`。
   - 新增长期运行的 `map_odom_authority`，并为它加独立文件锁。
   - 新会话一激活就立即清除旧 TF；只有会话、地图、点云时间、分数和位姿都合法的成功结果才能成为 TF 来源。
   - NavigationManager 和地图切换控制器要求连续 3 个新鲜 TF 样本与本轮定位位姿一致。仅看到历史 TF 不再算初始化成功。

4. **地图和路线一致性**
   - ProcessManager 状态响应包含实际栈地图和定位会话；运行中的旧地图栈不能被新任务复用。
   - 每次发目标前及真正调用 `send_goal_async` 前各检查一次 `waypoint.map_name == current_map`。
   - 路线规划时删除“两个普通点直接跨地图”的非法边；最终路线中的每次地图变化必须经过 `type=4` 切换点。

5. **地图切换使用先验位姿**
   - `MapSwitchTrigger.switch_pose` 传入新地图的 `nav2_init_pose`。
   - 新地图重定位先在切换点周围 `+-1 m`、航向 `+-30 deg` 的有限范围验证，不再首先进行无约束全局搜索。
   - 先验附近不能得到合格分数时切换失败并回滚，不使用随机全局结果继续行驶。

## 本次现场残留与清理结果

清理前发现三组 `re_localization/pcl_localization` 和两个 `nav2_container`。2026-11-17 20:38 已运行以下脚本完成清理，返回码为 0：

```bash
cd /home/akun/workspace/Car_jetson
bash navigation_tools/stop_navigation.sh
```

最终结果为导航进程 0、导航节点 0、ROS 2 CLI daemon 0、独立 Fast DDS discovery 0、Fast DDS 可见 FD/mmap/内核锁 0。共享内存中仍可见 55 个对象文件，但没有进程持有、映射或加锁；传感器仍运行时按设计不删除这些文件。完整记录位于 `navigation_monitoring/shutdown_checks/2026-11-17_20-38-59/`。该步骤没有发布驻车命令，也没有停止传感器或底盘驱动。

## 构建和离线验证

已完成以下包的构建：

- `multi_map_navigation_msgs`
- `re_localization`
- `nav2_init_pose`
- `multi_map_navigation`

已覆盖的离线场景包括：重复实例、父进程退出但子进程残留、未纳管 PID、关闭响应失败、状态未知、启动阶段逐项回滚、旧会话结果、旧点云、旧 TF、地图不一致、非法跨地图路线和地图切换先验传递。

## 下一次实车验收

1. 先运行 `stop_navigation.sh`，确认导航和 Fast DDS 残留为零。
2. 重新 source 四个工作区的最新 `install/setup.bash`，只启动一套管理栈。
3. 执行同地图连续两任务，确认第一任务完成后四个子栈均为 STOPPED，第二任务取得新 UUID。
4. 执行一次跨地图任务，记录 `/localization_session`、`/localization_result`、`/tf`、进程状态和地图切换消息。
5. 人为保留一个旧定位进程做负向测试；新任务应明确报告残留 PID并拒绝启动，不应发送 Nav2 目标。
