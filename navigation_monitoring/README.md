# 导航监测目录

本目录统一保存导航监测程序和每次任务的证据。

- `nav_task_monitor.py`：导航任务监测程序。
- `archive/2026-08-06/reports/`：凌晨三次测试总结。
- `archive/2026-08-06/raw/`：监测原始 JSONL。
- `archive/2026-08-06/logs/`：导航管理栈和 Nav2 日志。
- `runs/<日期_时间>/`：`start_navigation.sh` 自动建立的任务目录。

`navigation_tools/start_navigation.sh` 默认使用观察模式，不发布监测器保护性速度或故障命令；现场有人工作为车辆保护时保持这个默认值。它仍拒绝直接复用已有监测进程，必须等本次监测节点、启动记录和新鲜 IMU、合并激光、底盘数据就绪后才会派单。15 秒内未就绪会退出而不派单，管理栈和监测器可能仍在运行，应核对对应 PID 和任务再清理。只有明确需要旧的软件温度/内存保护时，才设置 `NAV_MONITOR_ENABLE_PROTECTION=1` 或直接运行 `nav_task_monitor.py --enable-protection`。

手动监测时直接运行 `nav_task_monitor.py` 默认会持续记录，即使导航 fault/idle 后也不会因为终态计时器自动退出；只有操作者明确结束测试并停止监控，才结束本轮录包。需要旧的“单任务终态自动收尾”行为时，显式加入 `--stop-on-terminal`。`start_navigation.sh` 是自动派单入口，已显式启用该参数。

监控器还会被动统计现有第一组前雷达过滤条件，并把跨测试结果写入 `filter_retirement_tracker.json`。连续至少 3 次测试且累计至少 3600 秒零命中时只记录删除提醒，不会自动修改过滤配置。规则见 `FILTER_RETIREMENT_POLICY.md`。

每次正常自动派单会生成 `monitor.raw.jsonl` 和 `monitor.log`。如果该次需要启动管理栈，还会生成 `management_stack.log` 和对应 PID 文件。Nav2 自身日志也统一写入 `runs/`。113到107的现场操作、监测和停止步骤见 [实地导航流程](../navigation_tools/FIELD_TEST_113_TO_107.md)。

## 室内 CPU 温度测试

`thermal_cpu_test.py` 是独立于 ROS 和车辆的受保护 CPU 温度测试器。它只创建本地 CPU worker，不启动导航、不发布 ROS 话题，也不控制底盘。每次测试写入：

```text
thermal_tests/<日期_时间>/
  metadata.json   # CPU、温度区、风扇请求状态、频率和测试参数
  samples.jsonl   # 每 0.5 秒一条原始样本
  samples.csv     # 便于表格/绘图分析
  run_state.json  # running 表示测试未正常收尾，便于重启后判断
  summary.json    # 机器可读结论
  summary.md      # 人类可读阶段汇总
```

默认保护线为 `78°C`，警告线为 `70°C`，启动负载前要求 CPU 最高读数不高于 `55°C`。达到保护线、读不到 CPU 温度，或父进程退出时，worker 会停止；达到警告线后脚本也会禁止自动进入更高负载阶段。每条样本还记录 Linux `boot_id`，便于识别硬重启。`78°C` 明显低于该 i7-8700 的 `100°C` 临界点；不要把保护线调到临界点附近。

先查看测试计划，不会产生 CPU 负载：

```bash
python3 navigation_monitoring/thermal_cpu_test.py --profile conservative
```

建议的室内顺序：车辆驻车、驱动轮抬离地面或断开底盘执行器，确认外置风扇已开启，且 `ps` 中没有导航进程后再执行。第一次只做单核，观察温度是否低于警告线：

```bash
python3 navigation_monitoring/thermal_cpu_test.py \
  --profile quick --execute --stop-after one_core \
  --ab-label external_fan_on --ambient-c 26
```

单核阶段稳定后，再运行物理核心阶段；仍不建议直接使用全部逻辑线程：

```bash
python3 navigation_monitoring/thermal_cpu_test.py \
  --profile conservative --execute --stop-after physical_cores
```

如果只想确认外置风扇下的静态温度，不启动任何 worker：

```bash
python3 navigation_monitoring/thermal_cpu_test.py \
  --monitor-only --duration 300 --execute \
  --ab-label external_fan_on --ambient-c 26
```

测试会在开始前拒绝检测到的导航管理器、重定位、LIO-SAM 或 Nav2 进程。不要使用 `--allow-navigation` 进行车辆测试；该选项只用于明确驻车且需要诊断的特殊场景。结束后优先查看对应目录的 `summary.md`，并把 `samples.csv` 与温度曲线一起比较外置风扇前后的峰值、升温斜率和节流计数。

只有在明确要验证 CPU/固件自身热保护时，才使用 `--hardware-protection-only`（别名 `--no-software-temp-stop`）。这个模式下 `70°C` 和 `78°C` 只记事件，不停止 worker，也不阻止进入更高负载；`100°C` 的 CPU 临界保护仍由硬件和固件处理。崩溃诊断时配合 `--fsync-interval 1`，样本至少每秒强制落盘。若机器重启后 `run_state.json` 仍为 `running` 且没有 `summary.json`，说明测试没有正常收尾，应结合最后一条 `samples.jsonl`、新旧 `boot_id` 和上一启动内核日志判断。

建议先运行 5 分钟 `monitor-only` 记录外置风扇开启时的静态基线；确认启动温度低于 `55°C` 后，再执行 `--profile quick --stop-after one_core --execute`。单核阶段峰值低于 `70°C` 且没有节流，冷却回到约 `55°C` 以下，才进入 `half_cores` 或 `physical_cores`。同一室温下分别用 `--ab-label external_fan_on` 和 `--ab-label external_fan_off` 记录“外置风扇开/关”，并通过 `--ambient-c` 填入实测室温；比较峰值温度、达到 `60°C` 的时间、冷却到 `50°C` 的时间和节流计数。不要以整机重启或达到 `100°C` 作为测试目标。
