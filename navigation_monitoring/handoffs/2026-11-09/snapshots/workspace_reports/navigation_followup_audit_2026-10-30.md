# 多地图导航模块复查

检查日期：2026-10-30。范围是 `multi_map_navigation` 活动源码、其 ROS 接口与 launch、以及充电衔接；历史文件 `navigation_manager copy.py` 不属于已注册运行入口。此报告针对上一轮已修复递归恢复、旧目标和全路径枚举后的当前源码。

## 已证实的高优先级问题

1. **切图结果没有任务/请求身份，旧结果会推进故障或新任务。** [navigation_manager.py:547](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:547) 的 `/map_switch_complete` 只传 `Bool`。回调只检查 `is_map_switching`，不核对 `stack_fault_active` 或导航代次，并先发布 `running` 再推进航点。隔离复现中故障状态下收到旧 `True`，状态被改发为 `running`，索引从 0 增至 1。建议把 switch ID/task generation 放进请求和结果接口，并在同一锁内验证状态后提交结果。

2. **地图切换可并发执行且忙时静默丢请求。** [map_switch_controller.py:74](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/map_switch_controller.py:74) 把订阅器放在 `ReentrantCallbackGroup`，但 `is_switching` 的检查与 `execute_map_switch()` 内赋值不是原子操作。双线程隔离复现两个请求都进入关闭和启动流程，分别启动 map2、map3。若后续请求观察到切换已在进行，代码直接返回而不发完成消息，调用端可能一直等待。建议用 `MutuallyExclusiveCallbackGroup` 或锁保护状态，并将忙请求返回明确、可关联的失败响应。

3. **切图工作不受任务故障取消。** 切图回调同步执行关闭、最多 600 秒 TF 等待和启动流程；没有接收导航任务代次，也没有在各阶段检查故障。替身复现故障标志在等待阶段置位后，旧切图流程仍启动了目标地图的堆栈。建议让切图成为可取消的异步操作，每次服务和 TF 等待返回后检查 switch token；故障时取消并清理部分启动的进程。

   此外，`navigation_timeout=300` 和 `map_switch_timeout=30` 都只有赋值、没有成员读取，配置期限并未真正限制操作。Nav2 目标请求或结果不返回、切图控制器退出不发完成消息时，管理器没有总期限退出。这是需要与任务隔离一同修复的 P1 问题，不只是参数命名优化。

4. **充电期间发生导航栈故障后无法通过现有接口恢复。** [navigation_manager.py:930](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:930) 在清理时置空 `waypoint_list`，但 `docking_hold` 保留；同时 `stack_fault_active` 保留为真。于是新任务被 [navigation_manager.py:178](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:178) 拒绝，`retry_docking` 和 `release_docking` 都因故障标志等条件失败。隔离复现确认现有两个恢复服务无法恢复该节点；重启可复位管理器内存状态，但 mux 停车锁存的解除也应纳入独立确认流程。保持停车是正确行为，问题是缺少明确的人工复位入口。

5. **Nav2 当前目标被取消后任务一直显示运行。** [navigation_manager.py:787](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:787) 清空目标句柄后调用 `abort_navigation(reason=0)`，而该分支只记录并返回，不完成任务、重发当前目标或进入暂停状态。隔离复现后 `is_navigating=True`、目标令牌为空、没有后续动作。需区分任务替换造成的旧目标取消与当前目标的意外取消，并为后者定义可恢复的状态转换。

6. **普通点之后的红绿灯点和未知类型会卡住任务。** [navigation_manager.py:839](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:839) 对 type 2 只推进索引后 `return`，没有灯态事件接续；其他未知类型也直接返回。MQTT 缺少 type 时默认生成 0，见 [mqtt_task_receiver.py:507](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/mqtt_task_receiver.py:507)。隔离复现 type 0 和 type 2 均保持 `is_navigating=True` 且无后续调度。入口应拒绝未知类型；红绿灯需要明确等待、放行和超时状态机。

7. **短时间重复任务会让规划器路线与执行路线不同。** [route_planner.py:87](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:87) 只读取最近一次 `/robot_state` 缓存。新的图在运行状态消息送达前可再次通过检查并覆盖图与 `path_nodes`。隔离复现中 planner 保存了 A-C-D，NavigationManager 仍在执行先前的 A-B-D；之后备用路线服务会对新图应用旧航点。QoS 1 允许重复投递，图消息没有 task ID。建议在规划器入口锁定“任务已发布/待接收”状态，使用唯一任务 ID，并让替代路线请求也带 task ID 和图版本。

8. **无效图边会抛出未处理的 KeyError；负权重会抛出 ValueError。** [route_planner.py:107](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:107) 覆盖重复 ID；[route_planner.py:128](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:128) 对边 ID 直接索引，无缺失检查。隔离复现缺失 endpoint ID 999 产生 KeyError。MQTT 解析也允许负权重和 NaN/Inf，见 [mqtt_task_receiver.py:498](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/mqtt_task_receiver.py:498) 与 [mqtt_task_receiver.py:561](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/mqtt_task_receiver.py:561)；负权输入传给 Dijkstra 后产生 ValueError，不在 `compute_path()` 捕获范围内。应在 MQTT 边界验证有限坐标、正/非负权重、唯一节点 ID、所有边引用、端点存在和合法航点类型，并在构图后再原子替换活动图。

9. **规划器的忙闲状态存在跨节点竞态且故障被当成空闲。** `RoutePlannerNode` 只有收到 `running` 等状态才拒收任务；`fault` 被映射成不运行，见 [route_planner.py:78](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:78)。与此同时 [status_reporter.py:241](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/status_reporter.py:241) 也把 fault 映射成 MQTT idle。新图可能在清理期间被发布，NavigationManager 仍处于故障清理而拒收，形成丢任务；上面的重复任务复现覆盖了状态送达延迟导致路线覆盖的情形。建议增加 `fault/cleaning/idle` 明确状态及任务接收确认，而不是把 fault 等同 idle。

## 已证实的进程与等待问题

10. **重复启动会覆盖进程句柄而遗留旧进程。** [process_manager.py:251](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/process_manager.py:251) 启动新进程后直接赋值 `self.processes['re_localization']`，没有先检查或关闭已跟踪实例。隔离复现 PID 111 被 PID 222 覆盖，旧进程无人再能通过该管理器关闭。相同模式存在于其他 `launch_*` 方法。服务应幂等：健康实例返回已运行；失效实例先验证并清理整个进程组，再允许重启。

11. **关闭 launch 父进程退出被当作整个进程组关闭成功。** [process_manager.py:428](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/process_manager.py:428) 只轮询根 `Popen`。根进程收到 SIGTERM 后若子进程仍存活，会立即 `_forget_process` 并返回成功，不再 SIGKILL 或核验子进程。替身确认只发 SIGTERM 就报告成功。应等待进程组消失或验证必需子节点退出，超时后再对仍存活的 PGID 发 SIGKILL。

12. **地图切换忽略停止服务结果。** [map_switch_controller.py:157](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/map_switch_controller.py:157) 每个关闭结果不检查，循环后固定 `return True`；服务异常或返回 `success=False` 也会继续检查“完全关闭”。替身中四项都失败仍返回成功。累积并检查每个响应，调用超时也立即返回明确失败。

13. **ROS 仿真时钟暂停时 TF 超时失效。** 导航管理器 [navigation_manager.py:409](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:409) 与切图控制器 [map_switch_controller.py:305](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/map_switch_controller.py:305) 用 ROS clock 判断操作期限。隔离测试冻结时钟并设置 1 微秒期限，八轮 TF 查询后仍未返回。操作期限应使用 `time.monotonic()`，而 TF 查询的时间戳继续用 ROS time。

14. **客户端等待超时没有取消服务端已经开始的操作。** [navigation_manager.py:376](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:376) 超时只返回 None，服务端的进程启动/停止仍可能继续；调用方随后恢复或清理时，迟到启动可能与新流程交错。建议把操作 ID 和任务代次传到进程管理器，检查每阶段结果并对过期启动做补偿清理。仅取消本地 future 不能撤销已经执行的服务请求。另已核对当前 ROS Humble 的 `Future.result()`：未完成时返回 None，不应将它误报成阻塞或 `InvalidStateError`。

15. **初始化阶段无限等待服务。** [navigation_manager.py:148](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:148) 的 10 秒只是单次等待，外层无期限循环；缺少 process manager 或 planner 时该节点永远不进入 executor。设置总期限、检查 ROS context，并在失败时明确退出或进入可诊断的降级状态。

## 状态上报与配置改进

16. **状态回调中的同步 ROS/HTTP 操作阻塞 executor。** [status_reporter.py:289](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/status_reporter.py:289) 最长同步等待 HTTP 10 秒；成功后在定时器回调里调用 `spin_until_future_complete` 关四个进程。真实隔离 ROS domain 226 的 mock 服务测试确认四个 future 在四次等待返回时都未完成，只在回调退出、executor 再次 spin 后才完成。生产实现应使用异步 HTTP/worker，并以 ROS future continuation 或独立 executor 完成进程关闭，不能在单线程 executor 回调内嵌套 spin。

17. **停驶计时没有在任务状态变化时重置。** `zero_speed_start_time` 仅在速度从非零变零时设置，见 [status_reporter.py:217](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/status_reporter.py:217)；任务从 idle 变 running 不会重置它。隔离复现先空闲停车超过阈值再开启任务，立即触发远程驾驶请求。计时应只累计“任务运行且非预期停车”时间，或在任务转换时重置，并排除 fault 状态。

18. **远程驾驶请求失败后本轮不再重试。** [status_reporter.py:285](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/status_reporter.py:285) 无条件把 `remote_driving_request_sent=True` 放在请求函数返回后；请求函数捕获超时/HTTP 错误但无成功布尔结果。失败后依赖速度跨越停车/运动状态才会清标志。应让请求返回成功状态，用配置的退避间隔重试，并避免阻塞 ROS 回调。

19. **部署与参数来源较脆弱。** `navigation_timeout`、`map_switch_timeout` 已声明读取但未用于操作截止时间；启动等待仍无限。`process_manager` 依赖源码绝对路径加载 YAML 与地图，见 [process_manager.py:30](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/process_manager.py:30)，其他启动器用 `get_package_share_directory`，方式不一致。`setup.py` 的 `install_requires` 只有 setuptools，`package.xml` 也缺少若干 Python/ROS 执行依赖声明。使用 ament share 路径/ROS 参数统一定位资源，补齐 rosdep 可解析的依赖声明并检查安装态启动。

## 已验证的其他逻辑缺口

- 相邻地图切换航点会错误翻转第二个切换点方向。[route_planner.py:378](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:378) 用前一个节点的 `map_name` 判断方向，却没有考虑前一切换点到达后的 `next_map_name`。隔离图 `map1 -> S -> map2 -> T -> map3` 中，T 的 approach map 变成 map3，而预期应为 map2。建议用路径遍历时显式维护当前地图，不要修改原始图节点方向属性。
- `navigation_timeout` 与 `map_switch_timeout` 当前无调用点，配置上看似有 300s/30s 保护，实际上 Nav2 结果等待无期限、TF 用固定 600s 且 ROS-clock 条件可停止推进。
- `navigation_manager copy.py` 是历史副本且未注册为 console script，需在发布流程中明确归档，避免人工运行错误文件。

## 可维护性与性能优化

- 普通航点到达后在 `on_goal_reached()` 休眠 6 秒，若导航堆栈健康，进入 `navigate_to_next_waypoint()` 后又休眠 6 秒，形成约 12 秒额外等待。应将它改成可取消的定时等待或明确的 costmap 就绪信号，并保留当前用于等待代价地图重建的目的；不要未验证就直接删除等待。
- `launch_new_stack`、关闭堆栈、future 等待在多个节点重复实现，超时与错误处理已产生差异。抽取统一的异步堆栈控制组件，让 NavigationManager 与 MapSwitchController 通过同一种操作接口调用。
- 把散落的字符串状态与多个布尔变量收敛为明确状态机，区分 `running/planning/switching/canceling/cleaning/fault/docking/idle`，并对非法转换记录原因和 task ID。
- 日志配置中的文件大小上限未实际用于 Nav2 日志轮换。限制日志保留数量/总大小，避免频繁重启后日志占满磁盘；日志记录 task ID、goal token、switch ID，减少全量航点的逐条 info 输出。
- `update_graph_switch_node_from_list()` 重复使用 `list.index()`，可改为 `enumerate()`；更关键的是保持原始图属性不可变，按路径生成方向视图，以免重规划受上次遍历修改影响。
- 删除未使用的重型导入与大块历史注释代码，归档历史副本；补齐依赖声明和干净环境下的安装验证。测试优先补 executor 并发、取消、故障清理和坏数据路径，而不是增加与实现逐行对应的测试。

## 范围与验证

- [隔离复现脚本](/home/akun/workspace/navigation_followup_audit_2026-10-30.py) 以 `__new__` 构造对象替身，确认 19 个当前源码观察。它故意断言旧行为，不能作为修复后通过的回归套件。
- [复现结果](/home/akun/workspace/navigation_followup_audit_results_2026-10-30.json) 保存 19 项结构化结果。
- [ROS callback probe](/home/akun/workspace/navigation_status_callback_probe_2026-10-30.py) 与 [结果](/home/akun/workspace/navigation_status_callback_results_2026-10-30.json) 在独立 ROS domain 226 使用两个最小节点和假服务，不连接 MQTT/HTTP，也不启动车辆或导航进程。
- 现有导航、进程健康、充电测试本轮使用源码导入再次通过 71 项；上一轮安装环境也通过 71 项。本轮只做只读检查与隔离探针，没有改业务源码，也没有做完整真实启动验收。
- 风险排序建议：先修切图请求关联/串行化/任务取消和进程组清理，再处理充电故障复位与取消目标状态；之后加严格输入验证、任务 ID 去重、状态计时和异步状态上报。每一类修复都补真实 ROS executor 的并发/超时测试。

复现命令（只运行替身和独立域的 mock 服务，不连接业务系统）：

```bash
cd /home/akun/workspace/Car_jetson
source /opt/ros/humble/setup.bash
source real_ws/install/setup.bash
source utils/install/setup.bash
source slam/install/setup.bash
source nav2/install/setup.bash
source multi_map_navigation/install/setup.bash
export PYTHONPATH="$PWD/multi_map_navigation/src/multi_map_navigation:$PYTHONPATH"
/home/akun/.venvs/car-host/bin/python3 /home/akun/workspace/navigation_followup_audit_2026-10-30.py
/home/akun/.venvs/car-host/bin/python3 /home/akun/workspace/navigation_status_callback_probe_2026-10-30.py
```
