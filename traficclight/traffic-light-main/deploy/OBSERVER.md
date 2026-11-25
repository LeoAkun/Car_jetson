# P4 独立逐帧观察器

入口为 `traffic_light_detector`，仅依赖既有 NumPy、OpenCV、PyTorch CUDA 和 TensorRT 10.7。CPU/mock 测试不导入 PyTorch、TensorRT 或 Ultralytics；实际 runner 使用专用 CUDA stream 和锁，每次返回独立的输出副本，GPU 执行失败后须重启 runner。

这是未标定候选模型的离线入口。结果中的 `GREEN` 表示当前帧识别颜色；是否满足本轮红转绿及起步预算，由后续导航状态机处理。所有输出固定带 `control_eligible=false`、`automatic_release_enabled=false`、`thresholds_calibrated=false`。`valid=true` 只表示 mock 来源下本帧存在唯一合格颜色框，不表示车辆获得通行许可。

旧 `app.py` / `utils/model/trafficlight.py` 保留作历史参考，含旧颜色锁存，未接入或部署到这个入口。新包没有 `light_status/green_count/red_count`，也不调用旧模型包装器。

## 接口和筛选

```python
from traffic_light_detector import Detector, FrameContext
from traffic_light_detector.engine import TensorRTRunner

runner = TensorRTRunner(engine_path, manifest_path, expected_sha256)
detector = Detector(runner)
observation = detector.detect(
    frame_bgr_uint8, (0.15, 0.10, 0.85, 0.90),
    context=FrameContext("offline-front", "mock-epoch", frame_seq, capture_monotonic_ns),
    received_monotonic_ns=receive_monotonic_ns,
)
payload = observation.to_dict()
```

- ROI 为原图归一化 `x0,y0,x1,y1`，满足 `0≤x0<x1≤1`、`0≤y0<y1≤1`；向内取整后裁剪，再等比例居中 letterbox 到 640×640，填 114，BGR→RGB、NCHW、FP32 /255。
- 输出按 `[1,300,6]` FP32 的 xyxy/confidence/class 解码，不重复 NMS。坐标用实际取整后的 resize 比例还原；`bbox` 是原图像素坐标，`bbox_normalized` 按原图宽高归一化，合格框均在 0–1 内。超出裁剪区域或落入 padding 的框不采用，也不将框裁小后继续使用；被排除的候选保留原始还原坐标供审计。
- 开发置信度阈值为 **0.5**，最大框面积为原图 **0.08**；两者均未进行真实交通灯标定。阈值可通过 `DetectorConfig` 显式传入，并随观察结果返回。
- 空检测、低分、超大框、ROI 外目标、冲突颜色、多个合格同色目标均为 UNKNOWN。多个同色框也可能来自不同方向的灯，当前无运动方向归属依据，因此不自动选择最高分。
- 达到阈值的非法类别或非法坐标会令整帧 UNKNOWN，不能忽略坏候选后采用另一个绿框。张量形状、dtype、非有限值或置信度范围错误也令整帧 UNKNOWN。
- `candidates` 保留最多 300 行候选及每行排除原因，`selected` 仅在唯一合格框时存在。坏张量没有可审计的候选列表；用 `reason` 记录错误。
- `FrameContext` 带 camera ID、source epoch、序号、源采集 monotonic；P4 仅接受合法 `source_mode="mock"` 上下文。缺少上下文或传入 real 来源均返回 UNKNOWN。P5/P6 必须先完成认证来源、重复/倒序/旧 epoch 和帧年龄验证，才能增加真实来源入口。
- `.150` 接收 monotonic 与源采集 monotonic 分别回传，不能跨设备直接相减。源序号的新旧验证属于后续接收层，逐帧 detector 本身不保存历史。`inference_ms` 包括 runner 的输入校验、复制、执行及回传，不含 JPEG 解码、预处理、后处理和网络。
- 该包不保存图片、不打印每帧日志，不打开相机、TCP 或 ROS 接口。

## 离线命令

在 `.150` 的独立 P4 源快照目录运行，下例对应 2026-10-29 已验证的版本：

```bash
cd /opt/traffic-light/builds/p4-observer-20261029-mIEa0HWq/source
PYTHONNOUSERSITE=1 /opt/traffic-light/venv/bin/python -m unittest discover -s tests -v
PYTHONNOUSERSITE=1 /opt/traffic-light/venv/bin/python -m traffic_light_detector \
  --engine /opt/traffic-light/builds/tl-20250724-trt107-fp16-20261029/artifacts/2025_7_24_fp16.engine \
  --manifest /opt/traffic-light/builds/tl-20250724-trt107-fp16-20261029/artifacts/model_manifest.json \
  --expected-sha256 0239cafccf2264fd43efc014fbf3a4aaec15930df46312d50aed6377ce78c05c \
  --synthetic-smoke --repeat 25 --roi 0.15 0.10 0.85 0.90
```

执行账户为 `trafficlight`；维护账户可通过已有 sudo/runuser 入口运行。不要改变账户 shell，也不需要重新安装依赖。`--image /绝对路径/图片.jpg` 可重复指定；默认报告每种输入的首个结果及候选数，`--include-candidates` 可输出全部候选。重复测试对每种输入的颜色、框、全部候选及 ROI 做指纹比较，变化或推理错误退出 1；正常的无检测 UNKNOWN 不算运行库故障。

部署记录见仓库根目录 `TRAFFIC_LIGHT_P4_OBSERVER_2026-10-29.md`。`current` 仍指向 P2，下一步是 P5 认证视频封装、最新帧缓冲及断流恢复；P3 真实金标和阈值标定仍待补齐。
