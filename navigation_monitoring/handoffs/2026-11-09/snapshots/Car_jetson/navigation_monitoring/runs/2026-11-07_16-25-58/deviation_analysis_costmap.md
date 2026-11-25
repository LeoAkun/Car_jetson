# 2026-11-07 实车 Nav2 / costmap 告警只读审计

对象：地图 `3b2mus`，任务 `task_1794039959`，113 → 106 → 107，run `2026-11-07_16-25-58`。时间均为北京时间 UTC+8。本审计仅读取存档日志、monitor JSONL、源码、安装配置及安装库；未查询 bag 数据库、未启动或停止 ROS、未修改任何源码或参数。

## 可以直接确认的结论

1. **“缓冲区内有效点数为 0！雷达数据在此处丢失。”是确定的调试代码误报警。** `obstacle_layer.cpp:468` 将 `total_points` 设为 0，此后整个函数从未增加或重新赋值；`:553` 永远走 `:558–559` 的 WARN。实际循环会正常处理观测、过滤点、写入 LETHAL，且会增加 `marked_points`，但这个成功计数与 WARN 判断没有关系。因此 227 次该告警不证明没有观测、不证明高度过滤清空，更不证明两台实际雷达全丢。安装的 `liblayers.so` 含该中文 WARN，而不含永不可达分支的“点数统计”字符串，与编译优化后的恒零判断一致。
2. **六次 Starting point 报错都发生在前往 106 的阶段。** 113 在 16:27:50 已成功到达；106 在 16:28:02 接受目标。这里的 starting point 是每次全局重规划时 TF 得到的车辆当前位置，不是任务最初启动位置、不是113航点，也不是对初始重定位“失败”的直接诊断。
3. **“lethal space”是一个宽泛的错误文案。** Hybrid-A* 调用当前全局 costmap 的碰撞检查。当前 YAML 全局没有 polygon footprint，使用 `robot_radius: 0.22` 和默认 `footprint_padding: 0.01`。此圆形模式下，中心格成本 `>=253` 即拒绝起点：包括内切膨胀区253、致命障碍254、以及 `allow_unknown:false` 时的未知255。不要求中心格一定是254，更不要求现场中心真有障碍。
4. **当前 source / installed YAML 是同一文件。** `nav2/install/nav2/share/nav2/config/nav2_real_params.yaml` 是指向 source YAML 的符号链接，内容 diff 为空，文件修改时间为 11月6日14:29，早于本次 run。launch 默认从该安装包路径读取，经 RewrittenYaml 改写行为树路径；没有发现本次另传一份 costmap YAML 的证据。启动日志只有WARN及以上，不能据缺失的INFO断言某个传感器没有订阅。
5. **不能把本次 Optimizer 失败直接归给旧的 retry fail_flag 缺陷。** 当前 `optimizer.cpp` 11月6日15:36已包含清除 fail_flag / critic缓存、每个 evalControl 独立重试计数、保留动态限速等修复；安装 MPPI库时间15:38，导出 `Optimizer::fallback(bool, unsigned long&)`，与修复后的签名相符，早于本次 run。三个报错仍表示当时没有得到可接受候选，但日志缺少具体是哪一个 critic、哪一个成本格造成拒绝的信息。
6. **另已确认独立的速度frame集成错误：LIO-SAM发布的导航系线速度未经旋转，被MPPI直接当机体系速度使用。** 下文列出完整调用链；这是比“也许雷达丢失”的猜测更具体的代码问题，但不能单凭它认定全部偏离及起点碰撞的唯一根因。

## 独立 Nav2 日志时序

主证据：`nav2_2026-11-07_16-26-35.log`；容器独立日志 `ros_logs/component_container_isolated_5451_1794039996124.log` 为同一批事件。

| 北京时间 | 主日志行 | 事件和含义 |
|---|---:|---|
| 16:26:37.854 | 15 | 控制周期1/30s小于模型dt=.05s；源码只警告，此时不会启用control sequence shifting。 |
| 16:26:51.125起 | 16起 | 恒零计数器误报警开始；全程227次。 |
| 16:27:18.127 | 49 | 第一次MPPI optimizer失败；当时仍在前往113，后来能恢复并到达113。 |
| 16:27:50.001 | 管理日志 | 113成功。 |
| 16:28:02.069 | 管理日志 | 106目标已接受。 |
| 16:28:08.990 / 10.067 | 110 / 114 | 两次起点被全局costmap拒绝，目标为(115.12,-143.71)。 |
| 16:29:11.916 / 12.461 | 193 / 196 | 再次出现相同起点无效。 |
| 16:29:43.820 / 50.034 | 237 / 246 | 两次MPPI失败，随后进入恢复行为。 |
| 16:29:50.075 | 248–250 | `Collision Ahead - Exiting DriveOnHeading` → `backup failed`，因局部costmap预测碰撞停止倒车恢复。 |
| 16:30:10.823 | 275 | SimpleProgressChecker超时，无足够平移进展。 |
| 16:30:26.755 / 27.681 | 296 / 300 | 最后两次起点被全局costmap拒绝。 |
| 16:30:43 | 管理日志 | 用户要求停止后进入终止流程。 |

全段另有43次30Hz控制循环超时。它说明处理没有满足目标周期，不能单凭此证明CPU过载或确定偏离根因。monitor在16:26:51–16:30:43的42个snapshot中，scan为29.50–29.83Hz（均值29.67），TF为约111–116Hz；在16:28:11起点错误附近，scan新鲜度.02s、map→odom .00s、odom→base_link .01s。频率/新鲜度并不证明点云几何或TF数值正确，但不支持“整个雷达/TF流中断”的解释。

## 实际插件、坐标、高度和成本条件

### ObstacleLayer及误报警

源码：`nav2/src/navigation2/nav2_costmap_2d/plugins/obstacle_layer.cpp:442–563`。

当前 local plugins 为 `[obstacle_layer,inflation_layer]`；global 为 `[static_layer,obstacle_layer,inflation_layer]`。各自只有一个 observation source `scan`，都是 `/laser/scan_merge_laser`，没有启用 YAML 注释中的可选 voxel / livox source。因此无需用“也许只是某可选sensor层为空”解释本次中文WARN：它来自两套必启的 obstacle层，但本身恒定误报。日志的logger只标local/global节点，没有source、plugin、观测数量，不能用于来源诊断。

流向是前后2D扫描 → merger点云`laser_link` → pointcloud_to_laserscan → `/laser/scan_merge_laser` → 各ObstacleLayer的TF MessageFilter → 投影点云 → ObservationBuffer → 各层mark/clear。源frame未显式配置时取消息header。local目标frame=`odom`，global=`map`。

实际run静态TF日志确认：base_link→laser_link=(0,0,.60)，front_laser=(.43,.28,.15), yaw45°，rear_laser=(-.43,-.28,.15), yaw225°。merger配置用ZOff=-.45将物理扫描高度表达在laser_link；但后续LaserScan重新投影的点位于LaserScan frame z=0平面，因而不应只凭前端物理高度认定costmap看到的z=.15。costmap最终高度取决于该LaserScan及TF到map/odom后的z。

source的height过滤在 `observation_buffer.cpp:103–155`：先转换完整点云到costmap frame，再按该frame的z过滤。local source配置[-5,10]，global source[-5,2]。**第二道mark过滤**在 `obstacle_layer.cpp:498–509`，读layer级 min/max。local并未设layer级height，实际默认[0,2]；global同样min=0且max显式2。所以local source max=10并不意味着最终10m高的点会被标记，source min=-5也不取消layer min=0。clear使用source过滤后的观测，先于第二道mark过滤执行。

mark再使用相对观测origin的3D距离，local `[.1,7)`m、global `[.1,2.5)`m，并要求worldToMap成功，才写LETHAL=254。clear raytrace上限分别8m、3m。local `inf_is_valid:true` 将正Inf改为range_max-epsilon来产生清除射线；global `false` 不这样处理，完全没有返回的方向不能指望它凭Inf清掉旧障碍。source `observation_persistence`、`expected_update_rate`未设，默认0：缓存保留最近一帧，且`isCurrent()`直接返回true，不提供丢帧超时保障。

source内`tf_tolerance:.5`不是当前ObstacleLayer读取的参数；源码使用costmap级`transform_tolerance`，local1s、global.5s，MessageFilter队列在源码固定50（YAML `message_filter_queue_size`未接入此处）。上述高度/TF配置差异是实际代码行为，不等于已证明它们造成这次偏离。

### 全局起点拒绝

入口：`nav2_smac_planner/src/a_star.cpp:217–219`；真正判定：`collision_checker.cpp:158–168`。圆形模式看当前global master costmap的起点中心格。`SmacPlannerHybrid::createPlan`将当前起点转换成map格坐标（YAML声明.05m，但静态地图YAML为.1m；StaticLayer可调整master尺寸，未保存global原始成本图，不能把声明值当作运行时分辨率），锁住costmap后做检查；`worldToMap`失败有另外的报错，本次不是那条。

global静态地图层→ObstacleLayer按max合并→inflation。clear footprint只清ObstacleLayer自己的网格 (`obstacle_layer.cpp:599–608`)，max合并不能把static层已有的致命值降成free。因此即使雷达看见车辆脚下为空、甚至进行了clear costmap恢复，static层障碍或其内切膨胀仍可使start拒绝。global inflation_radius=2m是衰减范围；它不把整个2m都变成“致命区”，253只在inscribed radius以内，外侧1–252只是软成本。

有理由继续检验的方向包括：定位/TF把当前车位投到静态障碍/未知格；当前或旧的scan点使附近格成为254并膨胀到车心；全局小圆形清除区没覆盖实际车体回波。这些都需同一时刻的pose、global costmap及分层数据才能区分。仅凭这六行日志无法在静态障碍、动态标记、未知255之间定案。

本轮并行静态地图审计提供了更具体的佐证：113及106航点中心均为free，113周围1m内无占据；但bag覆盖到的16:28:08.929位姿(111.576,-47.994)虽然中心仍free，距最近静态占据像素中心只有约.158m，已小于全局robot_radius .22m。这与16:28:08.990第一条lethal报错只差61ms，符合静态障碍膨胀后起点中心成本253的机制。由于没有同期global原始成本图，仍应称为强时间/空间佐证，而非唯一成本来源的完整证明。

### MPPI optimizer失败

`nav2_mppi_controller/src/optimizer.cpp:155–209`：打分失败后重采样重试，默认retry_attempt_limit=1；持续fail最终抛出文案。当前启用的 `CostCritic` (`cost_critic.cpp:143–180`) 与 `ObstaclesCritic` (`obstacles_critic.cpp:182–190`) 都会在所有候选轨迹碰撞时将fail_flag设true。CriticManager在前面critic失败后会跳过后续critic，因此WARN日志不能区分究竟是谁先置位。

local costmap的矩形footprint是x±.27/y±.43m（默认padding后x±.28/y±.44），CostCritic和ObstaclesCritic都consider_footprint=true；LETHAL以及局部不跟踪未知时的NO_INFORMATION构成碰撞。通常的低值膨胀仅增加软成本，不直接等于fail。没有证据表明path优化本体崩溃；这是控制失败和FollowPath abort，行为树会clear/retry/recover。

### backup failed

`nav2_behaviors/plugins/drive_on_heading.hpp:165–168,192–218`：BackUp沿当前朝向负方向模拟2s、10Hz，先检查当前姿态再检查后续姿态；行为树请求倒车.30m，速度.24m/s，模拟最多到剩余距离。`costmap_topic_collision_checker.cpp:45–84`用local_costmap/costmap_raw及published_footprint，footprint成本>=254拒绝。文案`Collision Ahead`由共享DriveOnHeading基类产生，倒车时也这样写；它不是“车前方真的有障碍”的专门诊断。

本次紧邻backup失败只有碰撞告警，没有“Current robot pose unavailable”或costmap/footprint取数异常，故最直接解释为局部costmap当前或倒车预测footprint碰到致命/未知成本。没有分层地图，不能证明是实障碍还是虚假标记。

### Failed to make progress

`controller_server.cpp:470–475`调用SimpleProgressChecker。`simple_progress_checker.cpp:56–95`只比较local costmap frame下XY平移距离；本次参数是20s内必须比baseline平移**大于**.3m，原地改朝向不算进展。16:30:10.823在16:29:50恢复失败后约20.75s发生，时间上符合该门限。它反映反馈中的平移不足，不代表定位直接判断路线错了，也不能单凭此区分底盘没执行、避障停住或位置反馈异常。

## 配置不一致与证据边界

全局为小圆r=.22m，局部为x长.54/y宽.86m的矩形；实测静态TF的前后扫描器安装点x已到±.43m，处在局部footprint的x边界之外。若真实车长沿base_link x，此模型明显值得核对：规划可能允许局部控制器不接受的间隙；车体回波也可能在局部清除矩形之外。但未读取车辆实物尺寸，不能据此直接断言“轴写反”或将所有失败归因于self-hit。

另有controller30Hz与model_dt=.05s不匹配，且43次循环超时；这个配置值得后续验证，现有日志不足以单独解释偏离。YAML中的TrajectoryValidator、CostCritic.trajectory_point_step、ax_*等部分字段在本地较旧MPPI实现中没有相应读取入口，不能按新版本文档假设其功能已经启用。

当前可以确定的代码错误是**雷达零点误报警**；当前可以确定的导航事实是**106阶段全局规划多次判当前位姿碰撞/未知、局部MPPI多次无可接受轨迹、倒车恢复也被局部碰撞检查拒绝，最终进度超时**。这些是失败链的观察结果。要锁定现场明明空旷为何成本图拒绝，必须结合已停止录制且覆盖相应时刻的历史pose/TF/scan/costmap；日志本身不能补出未记录的格子来源，也不能把后续人工驾驶段误当自主路径偏离的算法证据。

## 追加交叉审计：/odom速度frame没有在Nav2链路中修正

静态代码链可完整确认：

1. `slam/src/LIO-SAM/src/imuPreintegration.cpp:542,562–564` 从GTSAM `currentState.velocity()`直接写入twist.linear.xyz，child_frame_id在548设成`odom_imu`。本机`/usr/include/gtsam/navigation/NavState.h:41`明确该velocity成员是navigation frame，`:110`另有`bodyVelocity()`接口，但本发布代码没有使用。
2. 同文件TransformFusion `:120–125`从增量odom复制整条消息，只替换pose，保留原twist及child_frame_id后发布`odomTopic`；安装参数`odomTopic: /odom`。源码mtime为1月29日，安装LIO-SAM可执行mtime为10月15日，早于本次run。
3. `controller_server.cpp:197`创建的是`nav_2d_utils::OdomSubscriber`。`nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/odom_subscriber.hpp:68–91`默认订阅`odom`，回调仅逐字段复制twist.x、y、angular.z，不检查或变换child_frame_id。当前controller没有另配odom_topic，导航launch也没有odom remap。
4. `controller_server.cpp:478`取这条twist；`controller_server.hpp:196–202`只作最小速度阈值（默认.0001）；`nav_2d_utils/src/conversions.cpp:56–62`只转换2D/3D字段，均无旋转。
5. `nav2_mppi_controller/src/controller.cpp:143–144`直接传给evalControl，`optimizer.cpp:218`存为state.speed，`:284`将linear.x直接当作每条轨迹的第一个机体系vx；`:287`只有全向模型使用vy，Ackermann不使用该vy。`:345–365`才根据robot pose yaw把预测机体系vx积分到costmap坐标。

因此没有隐藏的twist TF补偿。并行bag审计提供的16:28:05样本：yaw≈1.1478rad、twist≈(.276,.693)m/s。若这些是导航系分量，平面旋转回机体系应有前向速度约.745m/s、横向约.033m/s，而当前MPPI接收到的前向只有.276m/s，少约63%。这是方向相关的初态速度错误，不能用“米/秒换算已修复”消除。

作用范围需准确描述：当前`motion_models.hpp:54–66`第一预测步使用实测初始vx，后续速度步直接来自候选控制cvx；故不能声称整个2.8s轨迹始终用错初速。错误会污染初始状态和首步碰撞/成本预测，是否足以造成观测到的偏离仍需修正前后的离线可重复比较。该速度问题与恒零计数器误报警彼此独立，也不自动解释全球地图上的起点碰撞。
