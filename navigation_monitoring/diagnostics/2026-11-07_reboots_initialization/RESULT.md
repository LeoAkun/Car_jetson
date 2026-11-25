# 两次异常重启与16:36初始化失败：当前结论

时间均为2026年11月7日北京时间。本轮只做日志取证和离线分析，没有启动ROS、传感器、导航、底盘控制或回放，没有修改系统和控制参数。用户已在本机终端输入sudo密码，完成只读系统日志导出。

## 范围按用户最新说明确定

- 分析第一次、第二次异常重启：新boot分别开始于16:16:07、16:19:30。
- 16:58左右为用户手动关闭，后续开机不计异常重启；这覆盖了此前“没有主动操作”的回答。
- 16:36重新发任务时仍在3b2mus地图覆盖区域，重点解释这一轮初始化失败。
- 17:04已回室内，用户明确要求该轮不算，排除本次户外初始化异常归因。其GPS等待日志保留作记录。

## 重启：确认整机异常中断，根因尚不能定案

| 事件 | 最后已保存主机活动 | 下一次系统启动 | 最后CSV温度 | 最后CSV可用内存 |
|---|---|---|---:|---:|
| 第一次 | 16:15:06.973 | 16:16:07 | 36°C | 27.64 GiB |
| 第二次 | 16:18:29.038（LIO进程出现） | 16:19:30 | 55°C（16:18:28.859） | 26.74 GiB |

这两组时间不是精确复位瞬间。boot ID改变、任务记录中断和wtmp开机记录确认主机重新启动。已保存资源记录没有持续高温、内存耗尽或CPU温度节流；两次均未观察到Nav2/MPPI运行，不能直接归因于路径优化失败或车速提高。

取得root导出的完整内核日志、每boot最后1500条系统日志以及pstore列表后：

- 未发现保存下来的kernel panic、Oops、OOM杀进程、MCE、致命/未纠正PCIe错误或系统过热关机记录；pstore与归档目录为空。这不排除没来得及落盘的故障。
- 两次旧boot均没有保存下来的正常系统关机/重启收尾。完整journal分别比独立CSV早结束约26秒、29秒，最后一段仍有取证空白。
- 存在真实的PCIe链路异常：`00:1c.2`持续报告`Correctable / Physical Layer / RxErr`。PCIe树确认该桥下为`02:00.0` Realtek网卡，对应`enp2s0`。它是排查线索，但没有证据证明它触发整机复位。
- 当前主机与本次boot日志的CPU均为i7-12700T；仓库10月8日满载报告写的是i7-8700，不能当成本台硬件已通过40分钟满载测试的证据。

现阶段应优先区分工控机DC输入/接头瞬断、主板/固件复位和PCIe链路问题，必要时再验证内存及无法落盘的内核故障。需要稳定独立电源对照、工控机DC输入瞬态记录和硬件逐项隔离才能继续缩小范围；本轮未执行这些测试，也未修改BIOS或PCIe节能参数。

## 16:36初始化：直接故障链已经明确

不是车离开地图，也不是一次真实点云配准算出了无穷大误差。

1. 重定位节点只完成`Configuring`，出现`/pcl_localization/change_state`响应超时，之后没有进入`Activating`。
2. 程序在激活阶段才装入PCD地图；本轮没有地图加载及ACTIVE记录。
3. 管理器仅检查启动进程还活着，仍宣告“重定位服务启动成功”。服务在配置阶段已经存在，初始化客户端因此开始搜索。
4. 2520次搜索请求都因地图未就绪提前返回，没有进入点云读取和NDT匹配。客户端只在服务success时更新分数，故始终保留`inf`占位值。
5. 客户端报PSO无有效结果后退出，管理器进入fault。LIO-SAM/Nav2未启动；同一boot的主机监控随后仍持续至16:46:05，说明初始化退出不等于整机重启。

**确定的软件缺陷是“进程存活被当作重定位已就绪”，以及服务未清楚区分未就绪与匹配失败。** 生命周期为什么没有继续激活，目前只能追到change_state通信超时，尚不能确定为DDS丢失、启动竞态或PCIe异常；不要将两个问题强行归为同一原因。

下一步软件修复应先确认生命周期ACTIVE、地图已装入及可用点云，再允许PSO请求；配置/激活需要有限超时、状态确认和受控重试，服务需返回明确的未就绪原因，并保留launch输出。此处仅分析，尚未实施修复。

## 详细证据

- [重启审计](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/REBOOT_AUDIT.md)
- [初始化审计](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/init_failure_analysis.md)
- [逐秒资源统计](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/reboot_resource_evidence.json)
- [sensor 代码与重启关系审计](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/SENSOR_REBOOT_REVIEW.md)：未发现可见代码主动重启主机；存在队列、串口异常和消息长度校验等稳定性风险，尚不能据此归因整机复位。仅静态检查，未启动驱动或修改运行配置。
- `system_logs/`内完整内核日志、系统尾部日志和pstore取证；`operator_clarifications.json`记录最新人工操作与室内范围说明。
