# 2026-11-23 统一 0.30 m / 5° 门限与五步离线验证

## 结论

五步已按指定顺序完成。生产 LIO 的 scan-to-map 平移/旋转门限已经统一为
`0.30 m / 5°`。第1、3、5轮隔离回放没有再形成三连拒，所有新回放的
`/odom` 相邻 5–15 ms 输出均未超过 `0.05 m / 1°`。一次已经建立 `/odom`
后的 280 ms IMU 间隔进入有界恢复，1.39秒后 readiness 恢复，没有发布
`/lio_sam/localization_fault`。第2轮失败观测的完整 PSO 回放仍明确失败，未产生
成功定位结果。

这些结果是离线验收，不替代实车验收。尤其是动态 costmap、真实执行器响应、
IMU/雷达物理时钟对齐和跨 epoch 原始增量来源仍需后续处理。

## 1. 实现

修改如下：

- `params_rslidar.yaml`：
  - `scanToMapMaxCorrectionTranslation: 0.30`
  - `scanToMapMaxCorrectionRotation: 0.087266463`（5°）
- `utility.hpp`：默认值同步改为 `0.30 / 0.087266463`。
- `imuPreintegration.cpp`：删除写死的 `0.15 m / 2°` 防御常量，TransformFusion
  直接读取与 mapOptimization 相同的参数，避免前端接受、融合层再次按旧值报错。
- `imuPreintegration.cpp`：已经输出合法 `/odom` 后收到预积分 reset，不再立即
  发布定位故障；清空旧 anchor，保留最后合法 `/odom`，等待匹配的新 epoch。
  readiness 超时仍由 ProcessManager 已有的5秒恢复门禁判定为进程故障。

构建：

- `lio_sam_imuPreintegration` 成功。
- `lio_sam_mapOptimization` 成功。
- 正式 install 中两个可执行文件均为指向本工作区 build 产物的 symlink；配置文件
  同样指向源码 YAML，因此下一次 LIO 进程启动会加载本次版本。

## 2. 第1、3、5轮旧/新门限 A/B 回放

回放只发布原始 `/imu`、`/vehicle_status`、`/rslidar_Pointcloud2` 和 `/tf_static`，
且分别运行于隔离的 localhost ROS domain。没有发布任务、`/cmd_vel`、刹车、驻车
或故障控制消息。

| 轮次 | 旧门限接受/拒绝 | 旧最长连续拒绝 | 新门限接受/拒绝 | 新最长连续拒绝 | 新 `/odom` 最大平移步长 | 新最大旋转步长 |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 214 / 11 | 10 | 206 / 0 | 0 | 0.005113 m | 0.117713° |
| 3 | 176 / 5 | 2 | 175 / 0 | 0 | 0.004013 m | 0.219093° |
| 5 | 169 / 6 | 6 | 151 / 0 | 0 | 0.004703 m | 0.176744° |

实时回放受调度影响，各次进入 mapper 的总帧数不是完全一致，因此不能把总接受
帧数当作确定性性能指标。确定性结论来自以下项目：

- 旧门限在三个窗口都重现拒绝，新门限三个窗口均为零拒绝。
- 对同时间戳可配对的样本，找到12帧“旧拒绝、新接受”。
- 三轮所有 5–15 ms `/odom` 对均为0次超过 `0.05 m`、0次超过 `1°`。
- 新接受 correction 与 IMU 消费的 optimized pose 在配对误差容差内一致。

临时产物：

- `/tmp/lio-zupt-timestamp.nHPZb0/gate15seq-{1,3,5}-rate1.0/`
- `/tmp/lio-zupt-timestamp.nHPZb0/gate30seq-{1,3,5}-rate1.0/`

## 3. 第4轮 IMU 间隔恢复

先原样回放第4轮：检测到一次 `0.270 s > 0.200 s` 的间隔，录制的
`/lio_sam/localization_fault` 为0。但该间隔位于首个融合输出之前，不能单独覆盖
运行中 reset 分支。

随后从同一第4轮输入生成只用于验证的变体：在 LIO 已经建立 `/odom` 后移除连续
27条 IMU，制造一次 280 ms 间隔。结果：

- 日志进入 `LIO_PREINTEGRATION_RESET_RECOVERING`。
- readiness 从 true 变为 false，1.39秒后重新变为 true。
- `/lio_sam/localization_fault`：0条。
- `/odom` 5–15 ms 最大平移步长：`0.000569 m`。
- `/odom` 5–15 ms 最大旋转步长：`0.002941°`。
- 超过 `0.05 m / 1°`：0次。

这验证了“单次短间隔不立即结束导航定位链”的代码路径。若5秒内不能恢复，生产
ProcessManager 仍会按现有 readiness 健康检查判故障；连续性门限和致命优化异常
处理没有放宽。

临时产物：

- `/tmp/lio-zupt-timestamp.nHPZb0/gapcurrent-4-rate1.0/`
- `/tmp/lio-zupt-timestamp.nHPZb0/gaprecover-4-late-gap-rate1.0/`

## 4. `/odom` 连续性与静态 footprint

使用 map2 生产静态地图和生产矩形 footprint（半长0.43 m、半宽0.28 m、额外
padding 0.07 m），审计12帧“旧拒绝、新接受”的修正：

- 12/12 optimized 目标位姿完整 footprint 为自由栅格。
- 每帧从原位姿到目标位姿取31个插值样本，全部为自由栅格。
- 5/12样本可与原始实车 TF 在250 ms内直接配对。
- 7/12发生在旧逻辑已经故障清栈、原始 TF 停止以后，使用同轮早期合法 TF 与
  replay mapping 轨迹重建。这7个样本证明静态地图合法，但不是实车同步 TF 证据。

审计程序：`audit_static_footprint.py`。

本项只验证静态 map2，不包含回放 Nav2 动态 obstacle layer，因此不能据此排除
前雷达自车回波或局部 costmap 短时出现253/254格。

## 5. 第2轮失败 PSO 回放

使用原第2轮时段的真实 `/rslidar_Pointcloud2`、GPS、map2 原始 PCD 和当前生产
重定位代码完整回放：

- GPS起点：`(-204.89, 83.20)`，与原实车失败轮一致。
- 30个初始化粒子完成，继续执行2轮迭代。
- 当前回放最佳 PSO 分数为 `0.2321`，仍明显高于有效阈值 `0.05`。
- 120秒截止后 `nav2_init_pose` 退出码为2。
- `/localization_result.success=false`。
- `fitness_score=inf`、`confirmation_count=0`。
- 未产生成功初始定位，不能通过生产启动门禁。

证据：

- `pso_trial_2/localization_result.yaml`
- `pso_trial_2/init_exit_code.txt`
- `pso_trial_2/logs/nav2_init_pose.log`
- `pso_trial_2/logs/re_localization.log`

本次门限修改没有更改 PSO、重定位 fitness 阈值或 nav2_init_pose 失败逻辑。

## 回归

- 静态 footprint、map authority、启动 footprint：21项通过。
- nav2_init_pose 自适应确认与 readiness：25项通过。
- ProcessManager LIO readiness/恢复健康检查：32项通过。
- ZUPT 车速历史 C++ 测试：1项通过。
- 合计：79项通过，0项失败。
- 目标文件 `git diff --check` 通过。

## 下一轮实车前仍需关注

1. 本轮证明 `0.30 m / 5°` 能消除已有录包中的三连拒，不证明所有实车场景都应
   接受这一量级的 correction。
2. 已确认的 IMU 与雷达约0.38秒相对时间偏差、跨 epoch 原始增量跳变仍未完成
   生产修复；新门限降低了失败概率，但不是这两个根因的替代方案。
3. PSO 第2轮仍按设计失败。要提高该类场景的成功率，应优化搜索种子和时间分配，
   不能通过把无效结果强行发布来解决。
4. 实车应先做静止2–5分钟，再做开阔区域低速直行、转弯和倒车。持续监控
   correction、readiness、`/odom` 连续性、map footprint 和动态 costmap。

原始实车录包全程只读，未修改或删除。
