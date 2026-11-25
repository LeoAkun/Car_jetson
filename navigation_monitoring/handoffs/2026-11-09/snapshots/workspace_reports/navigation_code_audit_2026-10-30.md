# 多地图导航代码检查

检查日期：2026-10-30（北京时间）。检查当前工作区源码，包含尚未提交的改动。

后续修复：同日已修复第 1 项的递归恢复和切图反复恢复问题。当前源码通过定时器重试，同一航点共享 `max_retries` 预算，并使用 `retry_delay`；重启成功不补满预算，推进航点后重置。预算耗尽进入现有 `fault` 停车及清理流程，任务结束或故障会取消旧定时器。相关导航、进程健康、充电测试共 47 项通过，包括独立 ROS domain 下的真实定时器测试。随后已将 `charging_dock_msgs` 和 `multi_map_navigation` 构建到项目实际 `install`，确认安装模块与源码一致，并从安装版本再次通过这 47 项测试。没有启动完整导航管理器或实车进程；真实定时器测试在独立 ROS domain 225 中使用最小测试节点。

第二轮修复：同日已处理第 2、3 项，并一并处理相关的第 7、8 项。导航任务使用递增代次，Nav2 目标绑定任务代次、航点索引和目标序号；状态查询、6 秒等待、动作服务等待及启动返回后校验任务身份，最终状态检查、`running` 发布与异步目标发送共用 `state_lock`。故障与任务结束立即使旧目标失效，过期目标若迟到接受则取消，过期结果、拒绝或异常不改变新任务。任务结束清理期间拒绝新任务。规划器改用 `nx.dijkstra_path()`，仅保存 `path1`；失败边删除后仍按原流程重新计算最短路。无路可达或端点不存在时清空旧结果，初次任务规划失败立即返回，不下发旧路线。

第二轮验证：新增 24 个用例，覆盖等待时故障及新任务替换、迟到接受/结果/拒绝/异常、正常航点推进与完成、清理期间新任务隔离、16 节点完全图只计算一条加权最短路、不可达时不复用旧路线。相关三组测试合计 71 项，分别通过源码导入和实际 `install` 导入验证；已重新构建 `multi_map_navigation`，确认导航管理器、路线规划器、充电模块、状态报告器安装副本与源码逐字节一致。语法及严重静态错误检查通过。未启动实车或重启正在运行的节点；已经运行的 Python 节点需重新启动后加载更新。

以下内容及行号保留初次检查时的记录。缺陷复现脚本刻意断言旧行为，修复后应运行工程中的 `test_navigation_recovery.py`，不应再把原始复现脚本视作通过标准。

检查范围为 `Car_jetson/multi_map_navigation/src/multi_map_navigation` 的导航管理、地图切换、路径规划、进程管理、充电衔接、任务接收及状态上报，并核对自定义 Nav2 行为树。历史文件 `navigation_manager copy.py` 不属于 `setup.py` 注册的运行入口。本报告不代表完整审计整个上游 Navigation2 源码。

初次检查结论：存在没有总次数限制的递归恢复调用，也存在路径枚举耗尽资源、任务停滞、故障后仍发送目标和异步回调串扰的问题。共整理 11 项，其中 16 个检查观察通过隔离执行当时源码确认。初次检查只新增检查脚本、结果和报告，没有修改业务源码，没有创建 ROS 节点或启动车辆、导航、底盘进程。后续修复状态见上方记录。

## 1. P1：恢复调用形成递归，`max_retries` 没有限制整条恢复链

位置：[navigation_manager.py:583](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:583)、[navigation_manager.py:287](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:287)、[navigation_manager.py:917](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:917)。

可以形成两条同步递归链：

```text
navigate_to_next_waypoint()
  → send_nav2_goal()
    → wait_for_server() 失败
    → abort_navigation(reason=2) 重启成功
    → navigate_to_next_waypoint()
      → ……

navigate_to_next_waypoint()
  → 检查状态未就绪，launch_new_stack() 失败
  → abort_navigation(reason=2) 内的 launch_new_stack() 成功
  → navigate_to_next_waypoint()
    → ……
```

`attempt_count` 是 `abort_navigation()` 的局部变量，每次进入都从 0 开始；一次启动返回成功就会绕过失败次数判断。只要外层问题持续，递归就持续增长。第二条需要状态检查和启动结果交替不一致。第一条可能出现在进程健康检查成功、但 ActionClient 所需的完整动作服务仍不可用时。

隔离复现中，`max_retries=3`，第一条模拟重启 497 次后触发 `RecursionError`；第二条模拟启动调用 1989 次后触发同样异常。这些是替身调用次数，没有实际重启车辆进程；具体次数受 Python 调用栈影响。

还有一条不增长调用栈的无总次数限制恢复链：[地图切换失败回调](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:561)在旧地图栈重启成功后，再次触发切图。连续注入 12 次失败，会再次重启并切图 12 次，未受 `max_retries=3` 限制。

建议：用任务/航点级持久计数限制整个恢复过程，通过定时器或状态机安排下一次尝试。达到上限时进入明确失败状态。`retry_delay` 目前也只有赋值，没有使用。

注意：当 `launch_new_stack()` 在同一次 `abort_navigation()` 中始终返回失败时，现有 `while` 的次数限制是有效的。问题在外层递归及跨回调恢复预算，不能简单描述为该 `while` 完全没有退出条件。

## 2. P1：故障发生后，等待中的旧流程仍可发送目标

当前状态：已修复，见上方第二轮修复和验证记录。以下保留初次检查说明。

位置：[navigation_manager.py:263](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:263)、[navigation_manager.py:573](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:573)、[navigation_manager.py:664](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:664)。

`navigate_to_next_waypoint()` 在状态检查后休眠 6 秒，醒来后直接发送已保存的航点。故障回调可以在其他线程把任务标为故障并清理，但发送函数和目标接受回调没有重新核对任务是否仍有效。

隔离复现：在这次休眠中将 `stack_fault_active=True`、`is_navigating=False` 并清空航点，随后原始发送代码仍向替身 ActionClient 发送了 1 个目标，并发布 `running`。

建议：为每次任务分配代号，在等待后、发目标前和目标响应时检查代号、任务活动状态及故障状态。过期但已被接受的目标应立即取消。仅在函数入口检查一次不能覆盖等待期间发生的故障。

## 3. P1：路径规划枚举并保存所有简单路径，复杂图可能长时间不返回

当前状态：已修复为单条 Dijkstra 最短路；没有增加计算耗时的硬截止时间，耗时仍随图规模增长。以下保留初次检查说明。

位置：[route_planner.py:365](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:365)。

`nx.shortest_simple_paths()` 返回生成器，但代码把它完整遍历，并将每条路径保存到 `self.path_nodes`。主路径下发和备用路线响应，都要等待全部枚举完成。

使用临时目录中的 NetworkX 3.4.2 实测：7 节点完全图保存 326 条路径，8 节点完全图保存 1957 条。按同一组合公式，12 节点完全图有 9,864,101 条起终点简单路径；此数字为计算值，未运行该规模压力测试。

这是有限图上的组合数量爆炸，不能称为数学上的死循环，但可以表现为长时间无响应、内存持续增长。外部服务等待超时也不会自动终止这里的计算。

建议：若只需要当前最短路线，使用单条最短路径算法；如需候选路线，设置明确的 K 值和计算预算。现有重规划已删除失败边，没有必要预先保存所有路径。

## 4. P1：航点导航超时参数未生效

位置：[navigation_manager.py:31](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:31)、[navigation_manager.py:628](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:628)、[navigation_manager.py:668](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:668)。

`navigation_timeout=300` 被声明、读取并赋值，但此后没有任何代码读取该成员。发送目标和等待结果都没有对应的超时处理。

因此，如果节点和生命周期健康、但目标响应或结果一直未完成，本管理器不会在 300 秒时退出。它仍依赖 Action 返回或其他外部故障/接管事件。行为树的有限恢复次数也不能替代整个航点的时间上限。

建议：为目标接受和航点执行分别设置单调时钟期限，超时后取消目标，再执行有次数限制的恢复或失败退出。

## 5. P1：地图切换没有完整的请求期限

位置：[map_switch_controller.py:27](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/map_switch_controller.py:27)、[map_switch_controller.py:245](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/map_switch_controller.py:245)、[navigation_manager.py:480](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:480)。

`map_switch_timeout` 同样只有赋值，没有使用；配置的 30 秒不会限制切换过程，其中仅 TF 等待就写死为 600 秒。导航管理器发出触发消息后，完全依赖 `/map_switch_complete` 回调。

控制器退出或没有返回完成消息时，管理器没有自己的切图超时处理，可能一直保持 `is_map_switching=True`、任务仍为运行状态。

建议：触发端和执行端都维护请求 ID 与期限，超时后统一失败处理，并拒绝过期完成消息。

## 6. P1：红绿灯和未知类型航点缺少后续状态转换

位置：[navigation_manager.py:781](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:781)、[mqtt_task_receiver.py:508](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/mqtt_task_receiver.py:508)。

对于非最终红绿灯航点，代码增加索引后执行 `pass`，没有发送下一目标，也没有建立绿灯事件或超时等待。未知类型走最后的 `else: pass`。MQTT 解析缺少 `type` 时默认填 0，会进入未知类型分支。

隔离复现两种类型后，都没有安排后续动作，也没有结束任务，`is_navigating` 仍为 `True`。红绿灯索引变为 1，未知类型停留在 0。

建议：红绿灯点进入显式等待状态，由绿灯事件恢复，并提供失败/等待期限策略；入口拒绝不支持的类型。此处不应直接用“遇到红绿灯就无条件继续”替代缺失逻辑。

充电点的 `docking_hold` 和 dry-run 不推进航点属于代码明确设计的保持行为，本次没有把它们作为死循环报告。

## 7. P1：Nav2 回调未关联具体任务和目标，迟到结果会污染当前任务

当前状态：随第 2 项一并修复，回调通过闭包绑定目标令牌。以下保留初次检查说明。

位置：[navigation_manager.py:637](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:637)、[navigation_manager.py:688](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:688)、[navigation_manager.py:738](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:738)。

回调只读取当前 `self.waypoint_list/current_waypoint_index`，没有任务代号、目标 ID 或航点快照。结果回调只检查 `stack_fault_active`；正常完成会把该标志复位，而新任务也会复位它。

隔离复现：清理后到达的成功结果导致 `'NoneType' object is not subscriptable`；对已切换为新路线的对象注入一个未识别归属的成功结果，会直接把新路线索引从 0 推进到 1。

建议：回调闭包绑定任务代号与目标 ID，只有当前目标才能推进状态；在清理、重规划、取消时使旧目标失效。充电分支已有 `docking_generation`，Nav2 分支可采用同类机制。

## 8. P1：新任务无路可走时可能下发上一个任务的旧路线

当前状态：随第 3 项一并修复，无路径或端点缺失时清空结果并停止发布。边引用和非法权重等输入校验仍需独立完善。以下保留初次检查说明。

位置：[route_planner.py:95](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:95)、[route_planner.py:139](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:139)、[route_planner.py:188](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:188)。

新任务重建了 `self.G`，却没有清空 `self.path_nodes`，并且忽略了 `compute_path()` 的失败返回值。随后继续读取并发布 `self.path_nodes['path1']`。

隔离复现：上个任务保存 `A→B`；新图中 A、B 存在但没有连接边。虽然 `has_path=False`，回调仍发布旧的 `['A', 'B']`。首次任务就不连通时，会因没有 `path1` 而异常。

建议：每次任务独立构造图和结果，验证端点、边引用及连通性；只有规划成功后才替换状态并发布。

## 9. P2：Nav2 取消结果没有结束或恢复任务

位置：[navigation_manager.py:716](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:716)、[navigation_manager.py:875](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:875)。

`CANCELED` 结果调用 `abort_navigation(reason=0)`，该分支直接返回 `True`，没有清理任务、进入暂停状态或安排继续导航。对于没有其他接续动作的当前目标取消，任务会空挂为运行中，并继续拒绝新航点列表。

隔离复现取消回调后，索引仍为 0、`is_navigating=True`，仅发布了 `running`。

建议：区分旧目标被替换产生的取消与当前任务取消；前者由任务/目标代号过滤，后者明确进入取消完成或暂停状态。

## 10. P2：TF 等待使用 ROS 时钟，仿真时钟停止时无法按墙钟超时

位置：[navigation_manager.py:388](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:388)、[map_switch_controller.py:305](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/map_switch_controller.py:305)。

两处都使用 `self.get_clock().now()` 判断期限。如果启用仿真时间但 `/clock` 缺失或暂停，该值不推进，循环条件持续成立。

隔离复现冻结 ROS 时钟、设置 2 毫秒期限，超过 8 毫秒且已执行 8 次查询后仍未按期限退出；由检查脚本的保护异常停止。此问题有条件：当前实车 Nav2 launch 默认 `use_sim_time=false`，不能据此断言当前实车一定触发。

建议：操作超时使用 `time.monotonic()`，循环检查 ROS 上下文和任务取消状态。TF 数据自身的时间戳仍使用 ROS 时间。

## 11. P2：初始化等待服务没有总期限或外层退出判断

位置：[navigation_manager.py:143](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py:143)。

三个 `while wait_for_service(timeout_sec=10.0) == False: pass` 中的 10 秒只是单次等待，外层会无限重试。路径服务或进程服务长期不上线，构造函数就不会完成，后续 executor 也不会开始 spin。

当前 ROS Humble 的 `Client.wait_for_service()` 内部会在上下文关闭后停止等待并返回就绪状态；外层没有 `rclpy.ok()` 判断。服务仍未就绪时，外层还可能不断立即重试。

建议：设置整个初始化阶段的截止时间，检查上下文退出，并报告缺失的服务。该项基于源码检查，没有在真实节点上制造服务缺失。

## 验证和运行版本边界

- 8 个活动 Python 源文件均通过 AST 语法解析。
- [检查脚本](/home/akun/workspace/navigation_audit_2026-10-30.py)读取当前类定义，移除 ROS 基类及模块导入，方法体保持原样，以本地替身替换服务、动作和时钟。不会执行节点构造函数。
- [检查结果 JSON](/home/akun/workspace/navigation_audit_results_2026-10-30.json)记录 16 项确认观察。它们是缺陷复现断言通过，不表示修复后的回归测试通过。
- NetworkX 3.4.2 只安装到 `/tmp/navigation-audit-20261030/deps`，没有安装到系统或工程环境。测试不代表指定生产 NetworkX 版本的性能基准。
- 初次审计时，系统 Python 缺少 NetworkX、Shapely 等依赖，`charging_dock_msgs` 安装产物也没有 `action` 模块，所以未运行原有测试。后续修复使用项目既有的 `car-host` 虚拟环境并构建充电接口，已通过上述 47 项相关测试。包内完整 `flake8` 和 `pep257` 检查仍分别报告 866、637 项风格问题，未在本次递归修复中清理；选定修改文件的语法及严重静态错误检查通过。
- 修复前，`navigation_manager.py`、`route_planner.py` 的源码与 `multi_map_navigation/install` 副本不同，`charging_navigation.py` 尚未出现在该安装副本中。修复后已构建 `charging_dock_msgs` 与 `multi_map_navigation` 到项目的实际 `install` 目录；逐模块检查确认导航管理器、充电模块、路线规划器、状态报告器都从 `install` 导入且和源码内容一致。`map_switch_controller.py`、`process_manager.py` 源码及安装副本原本一致。本次没有启动导航节点或实车验证。

在本次临时依赖仍存在时，可重复执行：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="/tmp/navigation-audit-20261030/deps${PYTHONPATH:+:$PYTHONPATH}" \
  python3 /home/akun/workspace/navigation_audit_2026-10-30.py
```

第 1、2、3、7、8 项已按上述范围修复并构建。后续仍需补齐第 4、5 项的动作及地图切换期限，第 6、9 项的航点和取消状态转换，以及第 10、11 项的时钟和初始化等待退出条件。
