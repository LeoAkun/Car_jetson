# Jetson 候选模型构建

当前候选 `2025_7_24.pt`：YOLOv10n，`0=red,1=green`。在 `.150` 的 TensorRT 10.7 / CUDA 12.6 原有环境构建，不使用旧 `export.py` 的 TensorRT API，也不全局替换 Ultralytics。

已完成的构建目录：`/opt/traffic-light/builds/tl-20250724-trt107-fp16-20261029/`。本地镜像为 `models/rebuilt/2025_7_24-trt10.7-fp16-20261029/`，详见仓库根目录 `TRAFFIC_LIGHT_P3_BUILD_2026-10-29.md`。

重建时先创建新的、由 `trafficlight` 拥有的构建目录，按以下布局复制源文件，保留仓库自带的完整 `ultralytics/`：

```text
BUILD_ROOT/
  source/
    ultralytics/
    scripts/build_jetson_model.py
    models/2025_7_24.pt
  artifacts/
  config/
  mpl/
  logs/
```

以下命令在 Jetson 上执行。`tl-rebuild-new` 是新目录示例，须先按上述布局准备并授权；不要复用已有产物目录。脚本拒绝覆盖已有 ONNX 和 engine。

```bash
BUILD_ROOT=/opt/traffic-light/builds/tl-rebuild-new
MODEL_SHA=9fae5374557eb3ce6f83bd43e8a34d8023f96ef46b91713855ce4d8788abdc36

for STAGE in export build verify; do
  sudo -u trafficlight env PYTHONNOUSERSITE=1 YOLO_AUTOINSTALL=false \
    YOLO_CONFIG_DIR="$BUILD_ROOT/config" MPLCONFIGDIR="$BUILD_ROOT/mpl" \
    OMP_NUM_THREADS=4 /opt/traffic-light/venv/bin/python \
    "$BUILD_ROOT/source/scripts/build_jetson_model.py" "$STAGE" \
    --source "$BUILD_ROOT/source/models/2025_7_24.pt" \
    --expected-sha256 "$MODEL_SHA" \
    --legacy-root "$BUILD_ROOT/source" \
    --output-dir "$BUILD_ROOT/artifacts" \
    --workspace-mib 1024 \
    --sample "$BUILD_ROOT/source/ultralytics/assets/bus.jpg" \
    --sample "$BUILD_ROOT/source/ultralytics/assets/zidane.jpg" \
    --sample /opt/traffic-light/current/test_data/offline.jpg \
    > "$BUILD_ROOT/logs/$STAGE.log" 2>&1 || break
done
```

重定向由当前 shell 执行，执行者也需能写 `logs/`。最后检查三个 stage 的报告均为 PASS，不能仅凭 shell 循环返回码判断。

`export` 核对 checkpoint hash、类别和有限参数，导出静态 640 ONNX（opset 13）并完整检查；`build` 验证 ONNX hash，构建 FP16、TF32 关闭、FP32 I/O 的 TensorRT 10 engine；`verify` 比较 PT/ORT CPU/TRT 并执行 100 次连续推理，生成 manifest。脚本只加载本地可信 checkpoint，不安装依赖、不修改源权重。

输入为 RGB FP32 NCHW /255、居中 letterbox 填 114；输出为 `[1,300,6]` 的 `xyxy,confidence,class_id`，one2one TopK 已在图内，无额外 NMS。ONNX metadata 的值为 JSON 编码；engine 为 Ultralytics 长度前缀 JSON 容器，须去头再交 TRT。

默认图片仅验证计算链路，没有红/绿金标覆盖。真实样本可通过重复 `--sample /absolute/path/image.jpg` 加入数值比较，但脚本不读取标注，**识别精度验收需另行对照人工标注**。manifest 默认保持候选状态、真实准确率未验收、自动放行关闭；不得据此自动切换 `current` 或启动车辆业务。
