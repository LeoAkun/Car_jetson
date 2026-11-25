# MPPI重试原生离线回归

测试直接调用真实 `Optimizer::prepare()`、`optimize()`、`fallback()` 及真实 `CriticManager::evalTrajectoriesScores()`，注入人工首轮失败、持续失败和计数评分器。真实噪声采样、模型滚动及成功后的控制更新均参与执行。它不初始化ROS、不创建节点或发布器，不发送车辆命令。

永久回归核心为项目内 `nav2/src/navigation2/nav2_mppi_controller/test/optimizer_retry_regression_test.cpp`，由该包的 `test/CMakeLists.txt` 注册为普通CTest；测试只使用修复后的新签名。历史对照使用本目录 `retry_harness.cpp` 的旧签名兼容快照。

在项目根目录执行：

```bash
source /home/akun/car-host-environment/project.bash
cmake -S navigation_monitoring/runs/2026-11-02_17-36-10/mppi_retry_fix_20261106 -B navigation_monitoring/runs/2026-11-02_17-36-10/mppi_retry_fix_20261106/build -DCMAKE_BUILD_TYPE=Release
cmake --build navigation_monitoring/runs/2026-11-02_17-36-10/mppi_retry_fix_20261106/build -j2
PYTHONDONTWRITEBYTECODE=1 python3 navigation_monitoring/runs/2026-11-02_17-36-10/mppi_retry_fix_20261106/run_validation.py
```

构建目录独立，未执行安装。`retry_before.json`、`retry_after.json` 保存各检查结果，`retry_validation.json` 同时记录对照、方法边界及源码/二进制SHA256，stderr日志保存真实优化器重置提示。旧版出现预期回归失败，修复版必须全部通过，runner才返回成功。

用例验证首轮失败后重新评分、持续失败的重试上限、零次重试、新周期预算独立、两个优化器交错预算独立、失败迭代提前停止、失败成本不更新控制、缓存清理、动态限速保留及位姿/速度/路径/模型/GoalChecker引用保留。

无节点回归在真实 `evalControl()` 的Twist打包前结束，因为该步骤需要从Costmap2DROS获取车体frame名称；使用相同的prepare/do-optimize/while-fallback结构。本测试不是完整 `evalControl()` 集成测试或闭环导航仿真。人工评分器不能检验真实障碍检测或确定11月2日首次碰撞判定原因，差速滚动模型也不能代替阿克曼底盘几何验收。
