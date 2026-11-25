# 阶段 5 低速自动对接实现说明

阶段 5 已提供一套独立的主动对接控制链。默认配置具有双重运动联锁，普通启动只做传感器、状态机和零速度输出验证。

## 组成

`fixed_trajectory_docking.launch.py` 只启动以下三个控制节点：

- `active_docking_controller`：唯一的 `/cmd_vel_dock` 发布者，组合里程计轨迹、后雷达停止和车辆状态安全门控。
- `command_mux`：在 Nav2 和对接指令间进行排他选择。
- `cmdvel_2_ecu`：使用对接低速配置，将 `/cmd_vel_selected` 转换到 `/ecu`。

不要再启动旧的阶段 3/4 独立测试控制器；阶段 5 只使用本说明中的 `fixed_trajectory_docking.launch.py`。

## 默认安全行为

配置文件 `config/stage5_docking.yaml` 默认设置：

```yaml
dry_run: true
publish_nonzero_command: false
run_distance_limit_m: 0.10
yaw_correction_enabled: false
lateral_correction_enabled: false
```

两个运动开关必须同时显式放行，控制器才可能发布非零速度。任何传感器超时、目标无效、障碍、车辆退出自动驾驶、轨迹越界或雷达/里程计不一致都会进入停车流程。故障清除后不会自动恢复运动。

## 构建和测试

```bash
cd /home/akun/workspace/Car_jetson/real_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding yunle_msgs \
  --packages-select yunle_msgs cmdvel_2_ecu charging_dock
source install/setup.bash
colcon test --packages-select charging_dock cmdvel_2_ecu \
  --event-handlers console_direct+
```

## Dry-run

```bash
source /opt/ros/humble/setup.bash
source /home/akun/workspace/Car_jetson/real_ws/install/setup.bash
ros2 launch charging_dock fixed_trajectory_docking.launch.py
```

另一个终端依次执行：

```bash
ros2 service call /command_mux/set_dock_active \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /active_docking_controller/enable \
  std_srvs/srv/SetBool "{data: true}"
```

状态应最终进入 `DRY_RUN_READY`，且以下话题必须始终为零速度：

```bash
ros2 topic echo /charging_dock/state
ros2 topic echo /cmd_vel_dock
ros2 topic echo /cmd_vel_selected
ros2 topic echo /ecu
```

取消并释放控制权：

```bash
ros2 service call /active_docking_controller/enable \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /command_mux/set_dock_active \
  std_srvs/srv/SetBool "{data: false}"
```

## 首次 0.10 m 动车测试

以下命令只有在现场急停、自动驾驶状态、低速方向和停车能力均确认后才可使用：

```bash
ros2 launch charging_dock fixed_trajectory_docking.launch.py \
  dry_run:=false allow_motion:=true run_distance_limit_m:=0.10
```

然后先请求对接控制权，再启动控制器：

```bash
ros2 service call /command_mux/set_dock_active \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /active_docking_controller/enable \
  std_srvs/srv/SetBool "{data: true}"
```

正常限距试验完成状态是 `LIMITED_RUN_COMPLETE`，不是 `SUCCEEDED`。只有后雷达进入最终停车窗口、车辆停止且连续验证通过后才会报告 `SUCCEEDED`。

## 放行前必须填写的现场参数

- `planned_reverse_distance_m`：按实际 `P0` 和最终停车位姿计算，不能直接照抄标称 `2.5 m`。
- `stop_trigger_distance_m`：必须包含实测制动距离和测量裕量。
- `sensor_x_m`、`sensor_y_m`、`sensor_yaw_deg`。
- `expected_range_m` 和 `expected_angle_deg`。
- `steering_sign`、`lidar_yaw_sign`。
- 固定 `P0` 已测量时，填写 `p0_target_*` 并将 `start_pose_validation_enabled` 设置为 `true`。

横向中心当前未通过专项验证，因此 `lateral_correction_enabled` 必须保持 `false`。航向修正也应在纯直线限距测试通过后再启用。
