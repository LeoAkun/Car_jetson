# 2026-11-14 三次代价地图/规划失败复盘

分析对象是上午 10:59、11:13、11:41 三轮任务。此次复盘只读取 ROS bag、Nav2 日志、导航管理器日志、持续观察器记录、地图及对应源码，没有修改车辆运行参数。

## 结论

三次都可归入“全局代价地图或全局规划不可行”，但不是同一个已经证实的根因。

| 任务 | Nav2 直接失败机制 | 根因判断 | 证据强度 |
|---|---|---|---|
| 10:59，110→114 | Hybrid-A* 从首次规划起反复耗尽 1,000,000 次迭代，18 次 `exceeded maximum iterations`；没有 `Starting point in lethal space` | 当时 master costmap/运动学约束使搜索不可行或搜索空间过于复杂。恢复动作又 9 次检测到前方碰撞。故障时刻没有完整 costmap/TF/scan 录包，无法把原因唯一归到 static、obstacle、残留格或位姿 | 失败分类高；具体障碍来源低 |
| 11:13，110→114 | 行驶中融合 `/odom`/TF 在 10 ms 内跳变 1.179 m，估计位姿被送入静态地图障碍；随后形成持续 `Starting point in lethal space` 并失败 | 上游 1.508 m LIO mapping correction 到达后，TransformFusion 重锚产生不连续输出。是运行中的里程计/融合状态跳变，不是启动重定位失败；已排除本次由已知前雷达近车体回波触发 | 高 |
| 11:41，map2 任务 | 已连续出 plan，规划起点推进约 22.8 m，随后 20 次 `Starting point in lethal space` | 失败位姿与静态地图边缘高度重合，足迹很可能压入静态障碍；但故障时段 rosbag 已被监控器提前停止，不能确定最终 master costmap 中是 static layer 单独触发，还是叠加了 obstacle 残留或 TF 瞬态 | 中 |

后两次都发生在定位、Nav2 和导航任务已经正常运行后。11:13 的 `map→odom` 在整包中保持常量；11:41 在首个 lethal 之前已有 59 条有效 plan。因此二者都不属于“启动时重定位没有成功”。

## 11:13：已闭合的故障链

这是三次中证据最完整的一次。专用 bag 覆盖 11:12:07–11:17:26，共 157,458 条消息，包含前后雷达、合并 scan、filtered scan、TF、`/odom`、LIO mapping odometry、全局和局部 costmap、plan、IMU、控制指令和车辆状态。

### 1. 终止任务的直接事件

关键时序如下：

| 时间 | 事件 |
|---|---|
| 11:16:38.299824 | 跳变前 TF：map 位姿约 `(151.704, -146.073)`；静态地图中心为空闲，足迹边界 30 个采样格全部为空闲 |
| 11:16:38.306530 | 新的 LIO mapping correction 到达；相邻 mapping 消息位姿步长为 1.508 m，消息 header 间隔 1.300 s |
| 11:16:38.310491 | 约 4 ms 后，`/odom`/`odom→base_link` 输出跳变；`/odom` 的相邻 header 只差 10 ms，平移却达到 1.179 m |
| 11:16:38.310491 | 跳变后的 map 位姿约 `(150.765, -145.347)`；静态地图中心为 occupied，足迹边界 30 个采样格中 23 个为 occupied |
| 11:16:39.073431 | 首个密集 `Starting point in lethal space`，全局 costmap 中心为 254，足迹边界 23 个 254 格对应静态地图 occupancy 100 |
| 11:16:39.309770 | 出现约 1.006 m 的反向 `/odom` 跳变，但足迹仍与静态障碍相交 |
| 11:17:06.390245 | 最后一条 lethal；全局中心仍为 254，13 个足迹边界致命格对应静态占用；随后 action 失败 |

车辆状态在首跳附近约为 1.8 km/h，控制速度约 0.35–0.56 m/s。车辆不可能在 10 ms 内物理移动 1.179 m。GNSS 同期约 0.96 s 只变化 0.51 m；IMU 保持 100 Hz、header 单调、数值有限且没有加速度或角速度尖峰。因此这是估计状态跳变，不是真实车体运动，也没有 IMU 丢帧或时间重置证据。

`map→odom` 在整包 1,795 个样本中保持 `(156.8051605, -192.3490295, yaw=1.5683263)`。真正跳变的是 `odom→base_link`/融合 `/odom`，所以不能称为 `map→odom` 重定位跳变。

### 2. 为什么会变成“当前位置在致命格”

Smac Hybrid 的 `Starting point in lethal space` 表示规划器建立搜索起点时，当前 map 位姿对应的车体中心或完整足迹碰到了 master global costmap 中不可通行的格子。它是规划起点合法性检查结果，不是重定位状态码。

本次大跳前，系统认为车在道路空闲区；大跳后，系统瞬间认为车向静态地图障碍内移动约 1 m。物理车辆没有瞬移，但规划器只能使用 TF 给出的估计位姿，所以搜索起点立即非法，无法生成新路径。

首个零散 lethal 其实早在 11:14:58.812 出现，当时足迹边缘只碰到 2 个静态 occupied 格，任务随后仍继续行驶。11:16:38 的 1.179 m 跳变把中心和大部分足迹送入障碍，才形成持续至任务失败的密集 lethal。对照的 11:22 同路线成功任务也出现过 10 次瞬时 lethal 和最大约 0.414 m/10 ms 的 correction-synchronous 小跳，但没有本次 1.179 m 正跳和 1.006 m 反跳，最终两个航点均成功。这说明系统平时已有融合不连续，失败轮的跳变量级足以越过道路净空边界。

### 3. 致命格来自静态地图，不是本次近车体雷达回波

密集失败期间，全局 costmap 中的 254 格逐格对应静态 PGM 的 occupancy 100；同一时刻局部 costmap 的车体中心和足迹边界全部为 0。局部 costmap 使用原始合并 scan 且不加载全局静态地图，这个对照把终止阶段的碰撞来源锁定为“估计位姿压入全局静态障碍”。

已知前雷达过滤窗口是原始角度 132°–135°、距离 0.60–0.83 m，并同时要求转换坐标落入给定矩形。11:13 的首个 warning 和最终密集 warning 期间：

- 合并后的 raw scan 与 filtered scan 最近有效点约为 2.17–3.04 m；
- 没有小于 1.2 m 的近车体点；
- 过滤窗口命中数均为 0；
- raw 与 filtered 的最近点相同。

前雷达原始 topic 在早段曾有 0.37–0.40 m 点，但转换到 `laser_link` 后半径约 0.299 m，小于 pointcloud-to-laserscan 的 `range_min=0.35 m`，没有进入合并 scan 和 global obstacle layer。因此，11 月 11 日曾观察到的 0.60–0.83 m 近车体回波不能解释 11:13 的最终失败。它可能仍是另一类风险，但不能拿旧录包结论替代本次证据。

### 4. 最可能的上游软件机制

LIO-SAM 的 TransformFusion 在收到低频 mapping odometry 后，以新的 mapping anchor 和缓存中的增量里程计计算融合输出；源码在 [imuPreintegration.cpp](/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/src/imuPreintegration.cpp:97) 订阅两个输入，在 [同文件](/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/src/imuPreintegration.cpp:128) 将增量放入队列、按 `stamp <= lidarOdomTime` 清理，再在 [同文件](/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/src/imuPreintegration.cpp:151) 用 `front⁻¹ × back` 乘新 anchor 发布 `/odom`/TF。IMUPreintegration 在 correction 到达后还会重置并重传播实时积分器（[同文件](/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/src/imuPreintegration.cpp:822)）。本包中，1.508 m mapping correction 到达后约 4 ms，融合 `/odom`/TF 就发生 1.179 m/10 ms 跳变，时序和数值高度一致。

当前最高概率机制是：异常偏大的 mapping correction 触发融合重锚，旧增量样本与新 anchor 跨 correction epoch 组合，使修正一次性注入 `/odom`/TF。IMU header 比 bag arrival 约超前 0.351 s；mapping correction 的处理延迟中位数约 0.436 s，本次异常 correction 约为 0.650 s。这会让队列内保留更多跨 correction 样本，可能放大问题。

这里仍有一个明确的证据边界：bag 录到了 `/lio_sam/mapping/odometry_incremental`，但没有录 TransformFusion 实际订阅的高频 `/odom_incremental`，因此不能直接指出其队列中究竟是哪一帧跨 epoch；也不能仅凭现有 bag 判断 1.508 m mapping correction 内部由哪组点云匹配、GPS 因子或特征退化产生。成功轮也有相似时基偏移，所以“时间戳偏移”不能单独作为根因。更准确的表述是：**异常大的 LIO mapping correction 加上 TransformFusion 重锚不连续，直接把规划位姿送入静态障碍；系统性时基偏移/修正延迟是可能的放大条件。**

同一系统的 11 月 10 日历史完整包提供了实现级旁证：该包同时录有 `/lio_sam/mapping/odometry`、`/odom_incremental` 和 `/odom`。10:37:57.665 的 mapping correction 后约 8.7 ms，`/odom_incremental` 自身跳 0.665 m/10 ms，而同 header 的融合 `/odom` 跳 1.232 m/10 ms；按当前源码中的 `mapping_anchor × inverse(queue.front) × queue.back` 公式重放，选取的五个大跳事件均可把记录的 `/odom` 复现到约 `10⁻¹⁴ m` 残差。全样本重放的最大残差仍有 0.146 m，因为 recorder arrival 顺序不等于 executor callback 顺序，不能据此还原每次回调的精确先后。10:37:57 事件中保留的 queue front 比 correction 的 recorder arrival 早 411.5 ms，back 比它晚 8.6 ms；队列只按 header 截掉不晚于 mapping anchor 的样本。若把同一输出改用 correction 前 anchor，重放残差为 0.372 m；改用新 anchor 后残差为 `3×10⁻¹⁴ m`。因此，跨 correction 边界保留队列并立即换用新 anchor 放大跳变是最高概率解释。历史消息没有 epoch 标签；它不是 11:13 的逐帧证据，且历史二进制版本和当前工作树未必完全相同，所以只作为同系统旁证。

## 10:59：规划搜索耗尽，不是已证实的 lethal-start

管理器在 10:59:21 拉起 re-localization，10:59:57 确认 `map→odom` 就绪，11:00:02 启动 LIO，11:00:20 Nav2 就绪并接受目标。首次规划在目标接受约 1.47 s 后失败，随后共出现 18 次 `failed to create plan, exceeded maximum iterations`，没有出现 `Starting point in lethal space`。恢复过程中出现 9 次 `Collision Ahead`，全局 costmap 清除服务多次超时，最终 11:01:39 action 失败。

Smac Hybrid 配置采用 Reeds-Shepp、最小转弯半径 1 m、64 个角度离散，并允许最多 1,000,000 次迭代。该日志表示搜索在当前 master costmap 和车辆运动学约束下遍历到上限仍找不到路径；它不等价于“当前位置一定在致命格”。离线静态地图检查中，当时起点、110、114 均为空闲，且位于同一空闲连通域，因此单纯的静态地图拓扑断开不是充分解释。动态 obstacle/inflation、近场障碍、位姿/朝向与运动学约束共同造成搜索困难的概率更高，但现有证据无法排序。

故障对应的专用 bag 只覆盖 11:00:45.948–11:00:46.655，只有 0.707 s，且位于首次失败之后；没有保存故障时刻的 global/local costmap、plan、完整 TF 与 scan。短包中前雷达过滤窗口没有候选点，后雷达合并数据里有约 0.60–0.78 m 的近点，能与恢复动作的 `Collision Ahead` 相容，但不足以证明它就是全局规划耗尽的原因。

## 11:41：静态边缘高度相关，但缺少故障时刻 rosbag

该轮 11:41:54 `map→odom` 就绪，11:42:14 Nav2 接受目标。持续观察器在首次 lethal 前记录了 59 条有效 plan，plan 起点从 `(-76.55, 177.45)` 移动到 `(-77.75, 199.55)`，约 22.8 m；因此定位和规划已经运行了一段距离。11:43:25 首次出现 lethal，之后共 20 次，11:43:43 曾短暂恢复一条 plan，11:44:09 最终失败。

把观察器中的 plan 起点和 `/odom` 拟合到 map2 静态地图后，首次 lethal 位姿附近最近 occupied 格距车体中心约 0.368 m，在车体坐标约为 `(x=0.173, y=-0.326)`。配置的 padded footprint 为 `x=±0.45 m, y=±0.30 m`；该格中心仅比侧边界多约 26 mm，加上 0.10 m 栅格尺寸后足迹很可能与静态障碍相交。拟合平均残差约 0.085 m、最大 0.231 m，所以这是强空间相关，不是逐格定层证据。

专用 rosbag 只录到 11:40:49，长包只录到 11:39:54；扫描工作区 462 个 `.db3` 后，没有任何 bag 覆盖 11:42:22–11:44:09。缺少故障时刻的 raw/filtered scan、TF、global costmap raw 后，无法区分 static layer、obstacle layer 残留、激光回波或 TF 瞬态对 master costmap 的贡献。

录包提前结束的原因已经查清。上一轮被监控器判定为 fault/idle 后持续 15 s，`nav_task_monitor.py` 设置 `shutdown_requested=True`；临时 supervisor 看到 monitor 退出，就同时向 observer、monitor 和 rosbag 发 SIGINT。rosbag 正常落盘，没有崩溃，也不是 30 s 分片参数造成。11:41 自动重试发生在这次联动停止之后。

针对这个数据完整性问题，已完成最小修复：`nav_task_monitor.py` 新增默认关闭的 `--stop-on-terminal`；手动监测默认只记录 terminal、继续监控和录包，并对同一连续终态只写一次 `monitor_terminal`。按现场人工保护约束，`navigation_tools/start_navigation.sh` 的保护性速度/故障发布也改为默认关闭，只有显式设置 `NAV_MONITOR_ENABLE_PROTECTION=1` 才启用。自动派单入口仍显式传入 `--stop-on-terminal`，保留旧的单任务自动收尾行为。相关测试结果为 `9 passed`，shell 语法检查和 Python 编译检查均通过。

## 额外发现

Nav2 obstacle layer 中的中文告警“缓冲区内有效点数为 0，雷达数据在此处丢失”目前不能作为雷达掉线证据。源码把 `total_points` 初始化为 0，却从未递增，判断分支会反复输出该告警；实际四路 scan 在 11:13 包中持续约 30 Hz，成功任务中也会出现同一告警。

全局 costmap 清除服务超时在三次失败中都会延长或恶化恢复过程，但它发生在初始规划/lethal 之后，应视为恢复链问题，不是 11:13 首次把位姿送入障碍的原因。

## 建议的处理顺序

1. **先修监控生命周期。** 手动测试模式下，terminal 事件只写记录，不自动结束 monitor/observer/bag；只有收到明确“结束测试”才停止。给旧自动单任务模式增加显式 `--stop-on-terminal` 即可保留原行为。否则下一次故障仍可能缺少最关键的后半段数据。
2. **补齐融合链录包。** 现有主题之外，必须明确录入 TransformFusion 实际订阅的高频 `/odom_incremental`，并保留两路 mapping odometry、`/odom`、`/tf`、IMU、GNSS、global/local raw costmap、plan、scan 和 `/rosout`，同时记录 header 与 arrival 时间。
3. **给状态跳变加保护和诊断。** 在 mapping correction 与融合 `/odom` 两级同时输出平移/旋转增量、header 间隔、arrival 延迟、队列首尾 stamp 和 correction epoch。检测到物理不可能的位姿步长时，暂停 planner/controller、标记任务无效并报警，由现场人工接管后重新定位；监控程序不发布驻车命令。
4. **重点审查 TransformFusion correction epoch。** correction 到达后应丢弃或明确重基准化旧增量队列，避免旧样本与新 anchor 组合。先用 11:13 bag 回放验证，再改车上参数。
5. **检查路线净空。** 11:13 首轮、11:22 成功轮和 11:41 都显示足迹经常擦静态障碍边缘。应按实际车体外廓、定位误差和 0.10 m 栅格误差检查路线，保留足够净空；不要通过全局删除真实静态障碍来掩盖状态跳变。
6. **修正 obstacle-layer 统计。** 正确递增 `total_points` 后，再区分“没有消息、消息为空、全部超范围、全部被过滤”，避免错误告警干扰现场判断。

## 可复核材料

- `task_1794626035_deep_audit/README.md`：11:13 完整证据链、时间对齐方法和限制
- `task_1794626035_deep_audit/jump_into_static_obstacle.png`：跳变前后静态地图与足迹叠加
- `task_1794626035_deep_audit/correlate_odom_jumps.out`：失败轮 correction 与 `/odom` 跳变明细
- `task_1794626035_deep_audit/correlate_success_jumps.out`：11:22 成功轮对照
- `task_1794626035_deep_audit/historical_cross_epoch_evidence.out`：11 月 10 日同时录有 `/odom_incremental` 的同系统 TransformFusion 旁证
- `lethal_events_analysis.json`、`lethal_events_summary.txt`：逐次 lethal 的 global/local costmap、静态地图和 scan 关联
- `task_1794627698_11-41_audit/README.md`：11:41 数据覆盖与证据限制
- `task_1794627698_11-41_audit/static_map_overlay_estimated.png`：11:41 估计位姿与 map2 静态边缘叠加

