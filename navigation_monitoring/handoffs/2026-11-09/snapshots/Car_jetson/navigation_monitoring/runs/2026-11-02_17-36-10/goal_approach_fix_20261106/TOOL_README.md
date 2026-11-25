# 原生评分器离线验证工具

`critic_harness.cpp` 编译并直接调用本机未修改的 `GoalCritic::score()`、`PathFollowCritic::score()`。派生包装仅设置受保护的参数；使用预先标记为无障碍的直线路径，仍执行真实的路径最远点计算。不调用初始化或配置方法，不创建 ROS context、节点、执行器、发布器或代价地图节点。

在项目根目录执行：

```bash
source /home/akun/car-host-environment/project.bash
cmake -S navigation_monitoring/runs/2026-11-02_17-36-10/goal_approach_fix_20261106 -B navigation_monitoring/runs/2026-11-02_17-36-10/goal_approach_fix_20261106/build -DCMAKE_BUILD_TYPE=Release
cmake --build navigation_monitoring/runs/2026-11-02_17-36-10/goal_approach_fix_20261106/build -j2
PYTHONDONTWRITEBYTECODE=1 python3 navigation_monitoring/runs/2026-11-02_17-36-10/goal_approach_fix_20261106/run_validation.py
```

Runner 读取当前实车 YAML、本目录 `before/` 中修改前 YAML 和转换器的既有物理低速补偿，输出 `critic_validation.json`，包含源码及二进制 SHA256。构建产物仅位于本目录 `build/`，不安装或替换导航库。

本次42项检查全部通过，3500个距离样本中原配置两项评分均为零的样本599个，新配置0个。精确浮点边界按真实运算记录，另验证边界前后0.0001米。候选顺序为前进、停止、倒退；目标在车后时倒退成本仍最低。

这是两项评分器对人工构造轨迹的验证。它没有运行整个MPPI优化器、所有评分器、障碍碰撞检测、底盘动力学或闭环仿真，不能证明实车振荡已经消除，也不能证明历史现象只有这一个原因。
