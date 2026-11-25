# 2026-11-07 两次重启前的温度与系统资源记录

本报告只读分析已有文件、用户级journal及wtmp，报告日期2026-11-07；未执行负载测试、车辆控制、驱动启动或服务修改。以下时刻全部为车载北京时间（UTC+8）。它记录本轮重启前的观测，不代表车辆当前状态。

两次断点对应boot ID依次为：

1. `271a95a2-06a6-4728-8955-9195feec0b76` → `7a56ea4e-c01f-4c9e-be99-4775707dd7f3`，关联导航目录 `2026-11-07_16-10-50`。
2. `7a56ea4e-c01f-4c9e-be99-4775707dd7f3` → `ca32f49f-62a3-4e77-aee9-af55c9ac556a`，关联本目录 `2026-11-07_16-17-52`。

boot ID变化、监控中断与新开机记录共同支持主机确实重启。现有数据没有显示持续高温、可用内存耗尽或已记录的CPU温度节流；这些观测不能排除供电瞬降、硬件复位、内核崩溃、PCIe/驱动或其他原因，也不能单凭监控自动标签确定根因。

## 最后有效的一秒级CSV样本

| 项目 | 第一次断点 | 第二次断点 |
|---|---|---|
| 旧boot简写 | `271a95a2` | `7a56ea4e` |
| 最后CSV样本 | **16:15:06.973** | **16:18:28.859** |
| CPU busy | 19.6% | 24.9% |
| CPU封装/最高核心温度 | 36 / 36°C | **55 / 55°C** |
| MemAvailable | 28,983,608 KiB，27.641 GiB | 28,037,528 KiB，26.739 GiB |
| SwapFree | 2,097,148 KiB | 2,097,148 KiB |
| load1 / load5 | 0.58 / 0.34 | 3.00 / 0.87 |
| 平均scaling_cur_freq | 861.7 MHz | 3600.0 MHz |
| package/core节流计数 | 0 / 0 | 0 / 0 |
| 下一boot监控开始/识别旧run时刻 | 16:16:17 | 16:19:37 |
| 最后样本至下次监控识别间隔 | 70.027秒 | 68.141秒 |

最后两项间隔包括重启过程、用户会话和监控启动等待，不能解释为停电时长，也不能用作精确复位时刻。`scaling_cur_freq` 是采样时的CPU频率读数，不是电压或供电质量记录。

原始文件：

- 第一次：[samples.csv](/home/akun/workspace/Car_jetson/thermal_testing/idle_crash_runs/20261107_124850_nav_field_test/samples.csv)、[state.json](/home/akun/workspace/Car_jetson/thermal_testing/idle_crash_runs/20261107_124850_nav_field_test/state.json)。CSV有11,947条数据；state仅记11,946条、最后时刻16:15:05.940，落后CSV一行。该倒数第二行CPU51.7%、37°C，不能替代最后CSV行。
- 第二次：[samples.csv](/home/akun/workspace/Car_jetson/thermal_testing/idle_crash_runs/20261107_161617_nav_field_test/samples.csv)、[state.json](/home/akun/workspace/Car_jetson/thermal_testing/idle_crash_runs/20261107_161617_nav_field_test/state.json)。CSV和state均128条，最后时刻一致为16:18:28.859。

第二次导航监控的最后一条六秒级资源快照在16:18:27.026，温度47°C；随后一秒级CSV在16:18:28.859记录55°C。因此重启前最后已保存温度应写55°C，不能仅引用47°C。导航监控16:18:29.038还保存了一条LIO进程出现的事件，所以16:18:28.859只是热监控最后保存时刻，不是主机最后存活时刻。

## 两个导航run时间窗内的资源范围

| 统计范围 | 第一次：16:10:50.246—16:15:06.973 | 第二次：16:17:52.758—16:18:28.859 |
|---|---:|---:|
| CSV样本数 | 249 | 36 |
| CPU busy峰值 | 51.7% | 37.9% |
| 封装温度范围 | 34—48°C | 36—55°C |
| 最高核心温度范围 | 33—48°C | 36—55°C |
| 最低MemAvailable | 27.386 GiB | 26.739 GiB |
| load1峰值 | 0.83 | 3.00 |
| load5峰值 | 0.36 | 0.87 |
| package/core节流计数 | 全部0 / 0 | 全部0 / 0 |

这些是导航run时间窗内的统计，未把第一次从12:48开始的整段空闲监控与导航混算。监控CPU比例来自整机`/proc/stat`差分，load来自`os.getloadavg()`；两者含义不同。记录仍不覆盖最后采样后的瞬时变化或板卡、供电模块温度。

## 是否有正常关机痕迹

两个旧run都未留下`finished_at`、`boot_id_at_finish`或`INTERRUPTED/COMPLETE`终态；下次开机监控把原`RUNNING`改为`ABNORMAL_REBOOT_DETECTED`。脚本的该标签只依据“旧run尚未收尾且boot ID改变”，不区分人工断电与异常复位，不能独立证明无人操作。

`last -x -F`中12:48及16:16的用户会话均显示`crash`；本次两次断点没有对应`shutdown system down`条目。用户级journal两boot末尾均突然截止，未发现监控收到停止信号或用户会话正常关机收尾的记录。这些情况与非正常中断一致，但wtmp/journal可能缺失或未持久化，不能单独作为硬件故障定论。

可见用户journal最后时间早于已落盘CSV：第一次16:14:39.063（热监控消息中的样本16:14:39.038），第二次16:17:59.994（样本16:17:59.968）。因此journal最后时间不能用作实际重启时刻。

当前账户不在`adm/systemd-journal`可读范围：`journalctl`明确提示看不到其他用户及系统消息，旧boot的`journalctl -k`返回`No entries`。这只说明本次查询没有可访问的内核条目，**不代表没有kernel panic、OOM、硬件/PCIe错误或正常系统关机记录**。该权限限制尚未解除，本报告没有据此排除这些原因。

## 自动监控的行为与完整性边界

本轮实际运行的是用户级 [carjetson-navigation-watch.service](/home/akun/.config/systemd/user/carjetson-navigation-watch.service)，以`--duration 86400 --interval 1 --label nav_field_test`启动 [idle_crash_monitor.py](/home/akun/workspace/Car_jetson/thermal_testing/idle_crash_monitor.py)，`Restart=always`。仓库中的`thermal_testing/carjetson-idle-monitor.service`不是本轮实际使用的单元。

脚本先写CSV并flush/fsync，再更新原子替换的state.json；第一断点CSV比state多一行符合这种写入顺序，但不能据此确定中断发生在哪条系统调用。每轮读取和持久化本身耗时，所以“一秒间隔”不是严格每秒一次。最后尚未保存的采样无法恢复。

第三boot的 [新run state.json](/home/akun/workspace/Car_jetson/thermal_testing/idle_crash_runs/20261107_161937_nav_field_test/state.json)显示监控16:19:37恢复；本次只读检查时服务为active/running。`loginctl show-user akun`为`Linger=no`，所以这是用户会话自动启动，不能保证登录前持续采样。自动恢复的是监控，不能表述为已自动恢复导航行驶。

本报告没有进行硬件负载测试，没有发送制动或其他车辆命令，没有修改或重启该服务。
