# MPPI内部失败重试修复

日期：2026-11-06。已完成源码修复、针对性回归、本机编译及安装更新。下次正常启动Nav2会加载新版共享库；本轮没有派发导航任务或发送车辆命令。离线通过不等于实车导航已验收。

## 问题及修复后的行为

原流程中，某项碰撞评分把 `critics_data_.fail_flag` 置为true，评分管理器跳过后续评分。fallback虽调用reset重新生成候选，却没有清理该标志；下一次评分管理器一开始仍看到true，直接退出。因此内部重试未真正重新评分，就耗尽上限并抛出 `Optimizer fail to compute path`。

已修复为：失败批次立即退出优化 → fallback清理该批次的失败标志、评分缓存、成本和候选状态 → 按现有预算重新采样并重新评分 → 成功后才更新、返回控制；如果仍失败，按原重试上限抛错。

具体修改：

1. `Optimizer::reset()` 清理fail_flag、最远到达路径索引和路径点有效性缓存。使用字段清理，保留当前位姿、实测速度、路径、GoalChecker、运动模型和CriticData引用。
2. `Optimizer::optimize()` 在评分失败时立即返回，避免对部分评分的失败批次更新控制，也避免 `iteration_count>1` 时继续处理失败标志下的后续迭代。
3. 重试计数由fallback内的函数静态变量改为每次 `evalControl()` 的局部变量，通过引用传给fallback，避免跨控制周期或优化器实例相互影响。对象布局不变。
4. fallback保存并恢复当前动态限速，避免reset恢复基础限制后让重试放宽已有约束。
5. 保留原重试预算。上限N表示初次尝试外最多重试N次；N=0首次失败直接抛错。先比较再递增，避免无符号计数溢出。

未在CriticManager或各评分器之间清除真实失败，也未降低碰撞成本、取消碰撞检查或增加重试次数。实际所有候选仍碰撞时，仍应报错；本修复不保证找到原本不存在的可行轨迹。

## 改动文件

- `nav2/src/navigation2/nav2_mppi_controller/src/optimizer.cpp`
- `nav2/src/navigation2/nav2_mppi_controller/include/nav2_mppi_controller/optimizer.hpp`
- `nav2/src/navigation2/nav2_mppi_controller/test/optimizer_unit_tests.cpp`：适配fallback计数参数。
- `nav2/src/navigation2/nav2_mppi_controller/test/optimizer_retry_regression_test.cpp`：永久回归测试。
- `nav2/src/navigation2/nav2_mppi_controller/test/CMakeLists.txt`：注册回归测试，仅测试target禁用有限数假设，确保 `std::isfinite` 检查有效；生产编译选项未改。

本轮开始前的文件原样保存在 `before/`，包括已有未提交修改；本轮补丁相对这些备份生成，没有覆盖为Git旧版本。CriticManager源码和Nav2参数与本轮备份一致，前一轮速度单位修复及GoalCritic=1.6的配置仍然保留。

## 验证结果

原生回归直接调用真实prepare、optimize、fallback及CriticManager评分分发，使用人工“第一次失败”“始终失败”及计数评分器。真实采样、模型滚动和成功后的控制更新参与执行。

| 验证 | 结果 |
|---|---|
| 修改前原样源码对照 | 23项中15项出现预期失败，复现问题 |
| 修改后原生专项回归 | 23/23通过 |
| 包内永久回归链接新构建库 | 通过 |
| 安装库专项回归（包括有效有限数检查） | 23/23通过 |
| 原有Optimizer单测 | 12/12通过 |
| 原有CriticManager单测 | 2/2通过 |

专项用例包括首轮失败后第二轮确实重新评分并成功、持续失败严格执行初次加N次重试、零预算、失败不更新控制、多个内部迭代遇失败立即退出、周期与实例计数独立、缓存清理，以及输入引用和动态限速保留。

独立回归不初始化ROS、不创建节点；它在完整evalControl打包Twist之前结束，因为打包需要读取Costmap2DROS的车体frame名称，不能称为完整evalControl集成测试或车辆闭环仿真。用例中的人工评分器验证失败流程，不能验证真实激光或碰撞栅格。原有单测会创建测试用节点，运行时已限制为本机及独立ROS domain 231，未启动车辆导航。

初次包内验收曾因环境优先加载旧安装库而出现新fallback符号缺失；明确切换至新构建库后通过。安装更新后，已用默认安装环境再次通过三项CTest，并用ldd确认专项测试加载的是安装目录的新库。有限数编译检查加固后，又单独重编并通过该专项测试。未留下未解决的测试失败。

## 安装与生效

已构建 `mppi_controller`、`mppi_critics` 及相关测试，并执行本包的CMake安装。安装头文件与源码一致，`libmppi_controller.so`已更新；详见 [安装核验](installation_validation.json)、[安装日志](install.log) 和 [动态库解析](ldd_installed.txt)。

不需要重启工控机；下次正常启动Nav2加载新库。若导航进程仍运行，它不会仅因为磁盘文件更新而自动更换已加载的库，需要正常结束该任务后再启动。本轮开始时未发现导航/控制相关进程，未在线重启任何车辆节点。

## 仍需实车确认的范围

本修复恢复了原有重试机会，不能唯一解释11月2日首次“所有候选碰撞”的原因，也不能代替低速、里程计速度坐标系、外参或代价地图验收。后续短距离航点复测应结合已完成的速度单位和近点评分修复，记录路径、完整odom、TF、速度请求/反馈、碰撞信息和动作状态。

原始结果、构建和校验材料：

- [原生对照](retry_validation.json)、[旧版结果](retry_before.json)、[新版结果](retry_after.json)。
- [安装库专项23项结果](installed_regression_result.json)、[安装环境CTest](ctest_installed.log)、[有限数检查复验](ctest_installed_finite_check.log)。
- [复现方法](TOOL_README.md)、[补丁](mppi_retry.patch)、[修改前清单](before_manifest.json)、[修改后清单](after_manifest.json)。
