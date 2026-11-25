# 2026-11-20 NumPy 2 环境变化、当日故障归因与近几日修改影响分析

## 1. 结论

当前车辆 ROS 环境中的 NumPy 版本变化，并不是导航工程自身的依赖升级。直接原因是 2026-11-19 晚上为了运行另一个工程 `my-driving`，执行了包含 `numpy>=2.2.6` 和 `opencv-python>=4.13.0.92` 的 `pip --user` 安装命令。

该命令把 NumPy 2.2.6 和 OpenCV Python 5.1.0.95 安装到：

```text
/home/akun/.local/lib/python3.10/site-packages
```

车辆 ROS Humble 也使用 `/usr/bin/python3`（Python 3.10），正常 Python 搜索路径会优先加载用户目录。因此系统 apt 安装的 NumPy 1.21.5、OpenCV 4.5.4、SciPy 1.8.0 和 Matplotlib 3.5.1 虽然都还在，但运行导航时实际看到的是用户目录中的 NumPy 2.2.6 和 OpenCV 5.1.0.95。

这是一次跨工程 Python 环境污染。`my-driving` 需要 NumPy 2.x，而 ROS Humble/Jammy 现有二进制扩展和 `transforms3d 0.3.1` 按 NumPy 1.x 构建或使用 NumPy 1.x API，两套要求不适合共用同一个用户级 Python 目录。

今天的问题需要分成三类：

1. **与 NumPy 2 高度相关的启动失败**：10:59～11:01 两次 `nav2_init_pose` 进程在启动后立即以 code 1 退出。旧安装代码导入 `transforms3d`，该导入在当前 NumPy 2.2.6 环境中可稳定复现 `np.maximum_sctype` 已被删除的异常。11:05 去掉依赖、11:06 重建后，11:09 同一启动阶段成功。由于当时子进程 stderr 没有独立保存，定级为“高置信度直接原因”，不是拥有完整原始 traceback 的绝对确认。
2. **与 NumPy 无关的导航算法故障**：11:09 是把限速跟随产生的累计 `map->odom` 滞后误判为单帧位置跳变；11:37 是车辆静止时 IMU 水平残差被预积分，连续激光修正超过 15 cm 门限。这两次都有录包或诊断数据支撑。
3. **与 NumPy 无关的环境加载问题**：`Package 'launch_real' not found`、`ModuleNotFoundError: multi_map_navigation_msgs` 来自 ROS overlay 没有完整 source，与 Python NumPy ABI 无关。

代码中删除 `tf_transformations/transforms3d` 只能绕开已经命中的路径，不能修复整个 Python 环境。Matplotlib、SciPy、`cv_bridge` 等仍处于不兼容或不受支持状态，下一次走到相关功能时仍可能失败。

## 2. NumPy 为什么会被改

### 2.1 `my-driving` 的依赖变更

`my-driving` 在提交 `4d41eea` 中把整个远程驾驶工作区从 Python `>=3.12` 调整为支持现场的 Python `>=3.10.12`。同一个提交把两个应用的 NumPy 下限从 `>=2.4.3` 调低为 `>=2.2.6`：

```text
commit: 4d41eea50b068be6fbca9dda2f6dce6255294628
time:   2026-11-19 16:31:38 +0800
title:  feat: support Python 3.10.12 across the driving workspace
```

相关依赖为：

| 工程 | 约束 |
|---|---|
| `apps/vehicle` | `numpy>=2.2.6` |
| `apps/driving` | `numpy>=2.2.6` |
| `packages/rd-webrtc` | `numpy>=1.26` |
| `rd-webrtc[opencv]` | `opencv-python>=4.13.0.92` |

这次提交的目的是让远程驾驶工程能在 Python 3.10.12 上运行。它没有隔离车辆 ROS Humble 使用的同一套 Python 3.10 用户目录。

### 2.2 实际安装命令

Shell 历史第 223 条记录了直接安装命令，核心内容是：

```bash
python3 -m pip install --user ... \
  'numpy>=2.2.6' 'opencv-python>=4.13.0.92' ...
```

执行前后历史都在运行 `rd_vehicle.main`，说明安装服务于远程驾驶工程，而不是导航工程。

安装目录时间戳显示 2026-11-19 18:16:29～18:16:32 安装了：

- NumPy 2.2.6
- OpenCV Python 5.1.0.95
- PyYAML 6.0.3
- aiortc 1.14.0、aioquic 1.3.0、av 16.1.0
- cryptography 50.0.2、cffi 2.1.1
- websockets 16.0 及其他 WebRTC/QUIC 依赖

NumPy 的 `.dist-info/REQUESTED` 文件存在，说明它是命令中直接请求的包，不是某个间接依赖静默带入。

### 2.3 为什么系统 NumPy 没被卸载，却仍然发生变化

系统包仍然完整：

| 包 | 系统版本 | 来源 |
|---|---:|---|
| NumPy | 1.21.5 | apt |
| SciPy | 1.8.0 | apt |
| Matplotlib | 3.5.1 | apt |
| OpenCV Python | 4.5.4 | apt |
| transforms3d | 0.3.1 | apt |

正常运行 `/usr/bin/python3` 时，用户 site-packages 位于系统 dist-packages 前面，所以加载结果变为：

```text
numpy 2.2.6  /home/akun/.local/lib/python3.10/site-packages/numpy
cv2  5.1.0  /home/akun/.local/lib/python3.10/site-packages/cv2
```

使用 `PYTHONNOUSERSITE=1` 验证时，加载结果恢复为系统 NumPy 1.21.5 和 OpenCV 4.5.4。这证明问题是路径遮蔽，不是 apt 包损坏，也不是今天重建 ROS 工作区生成了另一份 NumPy。

## 3. 已确认的兼容性变化

### 3.1 当前正常用户环境（NumPy 2.2.6）

| 模块 | 结果 | 影响 |
|---|---|---|
| `tf_transformations` | 硬失败 | `np.maximum_sctype` 在 NumPy 2.0 被删除 |
| `transforms3d 0.3.1` | 硬失败 | 同上 |
| Matplotlib 3.5.1 | 硬失败 | `_ARRAY_API not found`、`numpy.core.multiarray failed to import` |
| SciPy 1.8.0 | 可导入但警告 | 官方约束要求 NumPy `<1.25.0`，当前组合不受支持 |
| `cv_bridge` | 导入阶段报告 NumPy 1.x ABI 错误 | 顶层模块可能返回，但 C++ 扩展路径不安全，不能视为正常 |
| `cv2` | 加载成功 | 实际改为用户级 OpenCV 5.1.0 |
| `paho.mqtt` | 加载成功 | 来自原有用户目录，不是本次安装 |

`pip check` 报告无冲突并不能证明环境兼容。它只检查包元数据中的版本声明，不检查：

- 已编译扩展的 NumPy 1.x/2.x ABI；
- `np.maximum_sctype` 等已删除 API；
- apt 包与 pip 用户包混用产生的搜索路径覆盖。

### 3.2 屏蔽用户目录后的验证

使用 `PYTHONNOUSERSITE=1` 后，NumPy 1.21.5、SciPy、Matplotlib、`tf_transformations`、`transforms3d`、`cv_bridge` 和 OpenCV 4.5.4 全部恢复正常导入。

但 `paho.mqtt.client` 会变成 `ModuleNotFoundError`，因为当前 MQTT Python 包只安装在 `~/.local`。因此不能直接把整个导航启动环境改成 `PYTHONNOUSERSITE=1`，否则会把 NumPy 问题换成 MQTT 管理器启动失败。

### 3.3 为什么不能只把 NumPy 降到 1.x

用户目录中的 OpenCV 5.1.0.95 对 Python 3.10 明确要求 `numpy>=2`。如果只降 NumPy而保留用户级 OpenCV 5.1，会制造新的依赖不一致。恢复车辆 ROS 环境时至少要把用户级 NumPy 和用户级 OpenCV 当作一组处理；远程驾驶工程则应在独立虚拟环境中保留 NumPy 2.x/OpenCV 5.x。

## 4. 今天各次故障的因果划分

### 4.1 10:59～11:01：`nav2_init_pose` 两次启动即退出

任务：

- `task_1795143628`
- `task_1795143669`

共同过程：

1. `re_localization` C++ 节点正常启动并加载 map2 点云；
2. `nav2_init_pose` 启动后约 2 秒退出，code 1；
3. ProcessManager 按新故障门禁回收定位栈；
4. 没有进入 LIO、Nav2 和目标发布阶段。

当时安装的 `nav2_init_pose` 代码仍通过 `transforms3d.euler` 计算四元数。当前环境导入该模块会稳定触发：

```text
AttributeError: np.maximum_sctype was removed in the NumPy 2.0 release
```

11:05 左右源码改为使用 `math.sin/cos` 自行计算四元数，11:06:29 重建安装。11:09 的下一任务通过 `nav2_init_pose` 健康检查并完成重定位。

原始子进程的 Python stderr 未保存到独立日志，管理器只保存了 code 1。因此证据等级为：触发条件、旧代码依赖、可复现异常、修改时间和修改后恢复均吻合；高置信度由 NumPy 2 与 `transforms3d` 不兼容引起；缺少当时那一进程的完整 traceback。

### 4.2 11:09：不是 NumPy，属于 correction 判据错误

任务 `task_1795144184` 已完成重定位（fitness `0.011915`）、`nav2_init_pose`、LIO 稳定检查和 Nav2 启动。

失败点是 `map_odom_authority` 把限速发布值和目标值之间累计的 0.343133 m 滞后，当成一次 scan-to-map 跳变。真实单帧 correction 只有 0.042856 m，而且 LIO 已接受。

该问题已改成按真实 LIO correction 事件分类，中等 correction 做限速连续跟随，不再用累计欠账判断单帧跳变。

### 4.3 11:37：不是 NumPy，属于静止 IMU 预积分漂移

任务 `task_1795145873` 的重定位成功，fitness `0.011950`。车辆速度和 `/cmd_vel` 均为零，Nav2 目标尚未发送时，连续 scan-to-map correction 为 0.180865 m、0.169393 m、0.215357 m。

三次超过 0.15 m 硬门限后触发 `LIO_SCAN_TO_MAP_UNRELIABLE`。IMU 姿态补偿后仍有约 0.0151 m/s² 水平残差，旧预积分没有使用底盘静止反馈，启动负载又拉长 correction 间隔，最终形成明显预测漂移。

已增加只读 `/vehicle_status` 零速度更新；该修复已构建和离线回放验证，尚未完成下一轮实车闭环验证。

### 4.4 其他当日现象

| 现象 | 归因 |
|---|---|
| `Package 'launch_real' not found` | ROS underlay/overlay 没有完整 source |
| `ModuleNotFoundError: multi_map_navigation_msgs` | 同上，Python 包路径未加载 |
| 只 source 根 `install/setup.bash` 仍缺包 | 根 install 构建时记录的 underlay 链不完整 |
| Nav2 启动期间 RViz `-6` 退出 | 尚无证据归因 NumPy；Nav2 后续健康，非 11:37 直接原因 |
| 11:09 位置修正误报 | correction 分类逻辑 |
| 11:37 静止漂移 | IMU 预积分缺少零速约束 |

## 5. 近几日导航修改及可能产生的行为变化

当前工作区相对 Git HEAD 有 102 个已跟踪文件变化，约 10008 行新增、1801 行删除，并包含未跟踪新文件。它们跨越多轮任务，不能仅靠 `git diff` 精确还原每一行的修改日期。下面按诊断文档、构建时间和实际测试记录整理。

### 5.1 11 月 17 日：导航会话隔离和 ProcessManager

主要修改：

- 每轮定位增加 session UUID、地图、generation 和开始时间；
- 旧点云、旧 TF、旧 correction、旧 action 不能进入新任务；
- ProcessManager 保存和检查整个 PGID；
- 启动前扫描遗留重定位、LIO、Nav2 进程；
- 启动、清理和地图切换使用事务锁；
- 长启动/关闭从服务回调移到工作线程；
- 清理顺序固定为 `navigation2 -> liosam -> nav2_init_pose -> re_localization`；
- 状态未知、部分停止或存在未纳管进程时 fail closed。

预期收益是防止连续任务复用旧定位、防止两个导航栈重叠，并降低 ProcessManager 回调阻塞和永久 busy。代价是启动健康证据不完整时会更早失败，清理等待可能增长，外部手工启动的同名进程也会被当作未纳管残留。

### 5.2 11 月 18～19 日：重定位双确认和静态足迹门禁

主要修改：

- fitness `<0.025` 且先验、新鲜点云、完整 footprint 合法时直接接受；
- fitness `0.025～0.05` 时必须用新鲜点云二次确认；
- 两次位姿差要求不超过 0.30 m、3°；
- fitness `>0.05` 只允许刷新点云重试一次；
- 重定位单轮最长 120 秒；
- footprint 覆盖 occupied、unknown 或地图外时拒绝；
- 首目标前要求连续 10 个不同时间戳 TF 样本通过门禁。

预期收益是避免单帧偶然匹配、旧点云和车体压入静态障碍。代价是边界位置、窄路、unknown 较多处更容易失败，弱匹配会增加启动时间。双确认不是 11:09 和 11:37 的失败原因，两次都已经通过重定位。

### 5.3 11 月 18～19 日：位置跳变阶段 A～E

主要修改：

- correction 携带 instance、generation、epoch、source、residual、score 和拒绝原因；
- TransformFusion 只组合相同 identity/epoch 的 correction 和预积分；
- rejected correction 使用同时间戳预测位姿作为 anchor；
- 新 epoch 使用连续性 offset；
- `/odom` 相邻输出超过 0.05 m/10 ms 或 1° 时拦截并报告；
- 后端优化 frame 改为 `lio_map`，连续控制 frame 保持 `odom`；
- `map_odom_authority` 成为唯一 `map->odom` 发布者；
- 单帧 scan-to-map correction 硬门限为 0.15 m/2°；
- 连续三次 rejected 或退化触发定位故障；
- GPS factor 默认关闭；回环约束采用更保守的门限。

预期收益是阻止跨 epoch 拼接和大 correction 污染控制位姿。代价是保护更严格后会主动终止漂移任务；GPS 和回环更保守也会减少长期全局纠偏。correction 限速还会产生短时目标滞后，必须区分累计欠账和单帧跳变。

### 5.4 11 月 20 日 11:09 后：累计 correction 误报修复

主要修改：

- 不再以当前 `map->odom` 到目标变换的累计差值判断单帧跳变；
- 已被 LIO 接受的 5～15 cm correction 以 0.10 m/s 限速跟随；
- 每一步仍做静态 footprint 检查；
- 15 cm/2° 单帧硬拒绝保持不变；
- NavigationManager 的目标和地图切换四元数改用 `math.sin/cos`，移除 `tf_transformations`。

这会避免正常限速跟随被误报，同时绕开 NavigationManager 的一个 NumPy 2 崩溃路径。发布位姿会在 correction 后短时间平滑追赶目标，跟随速度过低时仍可能形成较长滞后。删除两个导入点不代表 NumPy 2 与整个 ROS 栈兼容。

### 5.5 11 月 20 日 11:37 后：静止零速度更新

主要修改：

- 连续 5 帧 `|speed| <= 0.10 km/h` 且 `/vehicle_status` 年龄不超过 0.25 秒时加入零速度因子；
- 零速度 sigma 为 0.05 m/s；
- 车辆移动或状态过期后立即停止使用零速约束；
- 只读状态，不发布底盘、驻车或刹车命令。

预期收益是抑制静止 IMU 小偏置积分。风险是底盘错误地长期报告零速时可能错误约束实际运动；新鲜度、连续样本和移动立即清零用于降低风险。当前只有离线回放结果，必须完成原地实车验证。

### 5.6 代价地图、雷达和车体边界

主要修改：

- 全局/局部 footprint 统一为约 0.86 m × 0.56 m，padding 0.02 m；
- 局部动态障碍 inflation 为 0.85 m；
- 全局静态障碍 inflation 为 0.75 m；
- cost scaling 均为 3.5；
- controller 频率从 30 Hz 调为 20 Hz；
- 前雷达特定窗口生成独立 filtered scan，仅供全局代价地图使用；
- 局部代价地图和近场保护继续使用原始 scan。

车辆会更倾向远离边界并更早对人和动态障碍产生代价。代价是窄路、靠边航点更容易无路可走，也可能增加停顿和绕行。如果过滤点其实是真实路沿，全局规划会低估该小区域；局部原始 scan 仍保留最后感知。第一组过滤目前仍有候选命中，尚不能删除。

### 5.7 底盘零速/制动输出修改

普通零速、非法输入和 `/cmd_vel` 超时不再设置 `Ecu.brake`；超时只发送一次 `motor=0 + N挡 + brake=false`，之后停止持续占用；人工模式停止占用底盘输出。

这有利于导航退出后人工接管。相应地，软件不再承担持续驻车保持，需要现场人工和车辆底层机制负责。测试聊天仍禁止发布驻车、刹车、零速和其他底盘控制命令。

### 5.8 ROS 环境自动加载

`~/.bashrc` 自动 source `navigation_tools/ros2_navigation_env.bash`，按顺序加载 ROS Humble 和各工作区 overlay。这解决新终端找不到 ROS 包的问题，但每个终端也会继承车辆 overlay。该脚本只处理 ROS prefix，不隔离 `~/.local` Python 包，因此不能解决 NumPy 2 污染。

## 6. 推荐修复顺序

### 6.1 先隔离 `my-driving`

为 `/home/akun/workspace/my-driving` 建立自己的 `.venv` 或使用项目现有 `uv.lock` 创建隔离环境。远程驾驶的 NumPy 2.x、OpenCV 5.x、aiortc 和 QUIC 依赖只进入该环境；禁止再对车辆公共 Python 执行 `pip install --user`。

先在隔离环境验证 `rd_vehicle.main --dump-config`、OpenCV/摄像头、WebRTC/QUIC、配置文件和串口导入。

### 6.2 恢复车辆 ROS 的 NumPy 1.x 组合

在远程驾驶隔离环境验证后，成组处理用户目录中的 NumPy 2.2.6 和 OpenCV Python 5.1.0.95，使 ROS 重新使用系统 NumPy 1.21.5 和 OpenCV 4.5.4。不要只降 NumPy 而保留 OpenCV 5.1。

保留或重新提供 `paho-mqtt 1.6.1`，因为导航 MQTT 目前依赖它，而系统 Python 中没有该包。应把纯 Python MQTT 依赖放入受控位置，避免重新引入 NumPy/OpenCV 用户级覆盖。

### 6.3 增加启动前环境门禁

建议在车辆导航环境检查中增加只读检查：

- Python executable；
- NumPy 版本和实际路径；
- OpenCV 版本和实际路径；
- `tf_transformations`、`cv_bridge` 和 `paho.mqtt` 导入；
- ROS overlay 包路径。

若车辆 ROS 环境检测到 NumPy 主版本大于等于 2，应在启动前明确报错并给出包路径。

### 6.4 回归验证

环境恢复后至少验证：

1. 导入 `numpy/scipy/matplotlib/tf_transformations/cv_bridge/cv2/paho.mqtt`；
2. `car_ros_env_check`；
3. `nav2_init_pose` 只启动检查；
4. 原地重定位和 2～5 分钟静止 LIO；
5. 短路线实车；
6. `my-driving` 在自己的虚拟环境单独启动。

恢复目标是车辆 ROS 的整套 Python ABI 一致，而不是逐个删除遇到的 NumPy 调用。

## 7. 2026-11-20 执行结果

上述隔离和恢复方案已经实施：

- 已创建 `/home/akun/workspace/my-driving/.venv`，未启用 system site-packages，也不会加载用户 site-packages；
- `.venv` 内固定使用 NumPy 2.2.6、OpenCV Python 5.1.0.95，并安装 `rd-core`、`rd-quic`、`rd-webrtc`、`rd-vehicle` 及其依赖；
- `.venv` 的 `pip check`、车辆端 `--dump-config` 和 LAN 配置 `--check-config` 均通过，检查过程未打开设备；
- `my-driving/start.sh` 已固定调用 `.venv/bin/python`；
- 公共用户目录中的 NumPy 2.2.6 和 OpenCV Python 5.1.0.95 已卸载；
- `paho-mqtt 1.6.1` 保留在用户目录；
- 普通 ROS Python 已恢复系统 NumPy 1.21.5 和 OpenCV 4.5.4；
- SciPy、Matplotlib、`tf_transformations`、`transforms3d`、`cv_bridge`、OpenCV 和 MQTT 均验证导入成功，公共环境 `pip check` 通过；
- 没有设置全局 `PYTHONNOUSERSITE=1`；
- 没有启动导航栈、传感器栈、录包器、监控器或远程驾驶设备。

迁移前后快照位于 `navigation_monitoring/diagnostics/2026-11-20_python_env_migration/`。11:37 的零速度更新仍需下一轮实车验证。

## 8. 关联材料

- `navigation_monitoring/handoffs/2026-11-20/START_HERE.md`
- `navigation_monitoring/diagnostics/2026-11-17_navigation_session_isolation_fix.md`
- `navigation_monitoring/diagnostics/2026-11-18_position_jump_root_cause_and_fix_plan.md`
- `navigation_monitoring/diagnostics/2026-11-19_phase_b_d_replay_report.md`
- `navigation_monitoring/diagnostics/2026-11-20_1109_navigation_failure_analysis_and_fix.md`
- `navigation_monitoring/diagnostics/2026-11-20_1137_stationary_imu_drift_failure_and_fix.md`
