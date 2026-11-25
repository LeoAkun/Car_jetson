# 阶段 3 相机与 AprilTag 验收记录

测试日期：2026-08-14  运行平台：当前 ROS 2 Humble 开发机  设备：`/dev/rear_camera -> /dev/video0`

## 测试方法

启动完整阶段 3 链路：

```bash
source /opt/ros/humble/setup.bash
source real_ws/install/setup.bash
ros2 launch charging_dock apriltag_docking.launch.py start_camera:=true
python3 real_ws/src/charging_dock/tools/stage3_camera_check.py --duration 10
```

同时使用 `ros2 topic hz`、`ros2 topic delay`、`ros2 topic echo` 和进程资源快照交叉检查。测试脚本只订阅话题，不发布速度或控制命令。

## 结果

| 指标 | 实测 | 阈值 | 结果 |
| --- | ---: | ---: | --- |
| `/rear_camera/image_raw` 频率 | 30.0 Hz | 不低于 20 Hz | 通过 |
| `/rear_camera/image_rect` 频率 | 29.8 Hz | 不低于 20 Hz | 通过 |
| 图像分辨率 | 1280 x 720 | 与标定一致 | 通过 |
| `/apriltag/detections` 频率 | 29.9 Hz | 链路应稳定 | 通过（空检测） |
| 感知链路时间戳延迟 | 新启动 5 s 窗口 P95 约 238–247 ms；长时间运行窗口约 862 ms | P95 不超过 150 ms | **失败** |
| 适配器数据新鲜度 | `stale_frame` 与输入帧数相同 | `data_fresh=true` 不低于 99% | **失败** |
| 有效目标标签观测 | 0 Hz | 不低于 15 Hz | 未通过/无法验收 |
| 目标标签关联率、位姿 RMSE | 未测 | 100%、10 mm、1 deg | 需真实标签和测量治具 |
| CPU 资源快照 | `usb_cam` 15.3%、`image_proc` 68.3%、`apriltag` 88.3%、适配器 7.8% | 记录并无持续降频 | 已记录 |
| 温度快照 | 约 27.8 / 42.0 / 66.0 C | 无热降频 | 已记录 |

当前相机画面是标定棋盘板，不是配置的 `tag36h11` ID 0，因此检测数组为空是预期现象；这不能替代 `0.3~2.0 m`、横向 `+/-0.30 m`、航向 `+/-15 deg` 的真实标签测试。

## 阶段 3 代码与依赖检查

- `apriltag_ros`、`apriltag_msgs` 和 `usb_cam` 均能从 ROS 2 Humble 环境找到；已记录版本：`apriltag_ros 3.4.0`、`apriltag_msgs 2.0.2`、`usb_cam 0.8.1`。
- 配置使用 `36h11`、ID `0`、尺寸 `0.16 m`，适配器发布项目内部 `DockTagObservationArray`，第三方检测消息未直接接入控制器。
- 配置注册表、重复 ID、族校验、质量门控逻辑测试共 16 个 Python 单元测试通过；包含版权、风格和文档检查在内的包测试共 19 个测试全部通过，包构建通过。
- 完整 launch 能启动相机、整流、AprilTag 和适配器；关闭 launch 后所有进程均正常退出。

## 结论与下一步

阶段 3 目前只能判定为**未完成验收**。频率、分辨率、依赖和适配器单元测试通过；P95 延迟、数据新鲜度和有效位姿输出未通过。主要阻塞是 USB/V4L2 图像时间戳和处理队列延迟，需在阶段 4 前处理（降低采集/整流/检测队列、确认 V4L2 时间戳基准，并在真实标签前后复测），不能仅把 `max_data_age_sec` 放宽来掩盖延迟。
