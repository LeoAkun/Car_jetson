# Jetson P2 运行环境

目标设备为 `192.168.1.150`，JetPack 6 / Python 3.10。

从保留 `scripts/`、`deploy/` 相对位置的代码目录运行：

```bash
sudo bash scripts/install_jetson_runtime.sh
```

脚本创建 `trafficlight` 系统用户（`/usr/sbin/nologin`）、`/opt/traffic-light/venv`（`--system-site-packages`）、`releases` 和 `shared/{config,recordings,logs,cache}`。再次运行会校验既有用户与路径，不替换不一致的用户、目录或 GPU 规则。不执行 apt/pip 安装或升级。

GPU 规则 `/etc/udev/rules.d/99-trafficlight-gpu.rules` 为该用户添加精确设备 ACL，保留 NVIDIA 的 owner/group/mode。规则覆盖 JetPack 6 的 `/dev/nvgpu/igpu0/{as,channel,ctrl,tsg,power}`、兼容 GPU 节点、`nvmap` 和 DRM render 节点。用户无需加入 `video` 组；不授予 GPU 调试/调度控制节点或本地摄像头权限。当前节点直接应用 ACL，后续设备创建由 udev 恢复。重启后的恢复验证尚待现场补做。

`requirements.lock` 当前没有额外 PyPI 依赖：P2 诊断只需要标准库和既有系统库。GPU/二进制库版本及路径单独保存到环境清单，不能通过通用 PyPI 版本替换。P4/P5 增加依赖时再审核并锁定纯 Python 包。此阶段不安装旧 Ultralytics 应用的训练/导出依赖。

部署后的自检入口：

```bash
sudo -u trafficlight env PYTHONNOUSERSITE=1 \
  /opt/traffic-light/venv/bin/python \
  /opt/traffic-light/current/scripts/self_check.py \
  --image /opt/traffic-light/current/test_data/offline.jpg
```

真实断网模拟使用独立网络命名空间，不断开 Jetson 的维护网络：

```bash
sudo unshare --net -- runuser -u trafficlight -- env PYTHONNOUSERSITE=1 \
  /opt/traffic-light/venv/bin/python \
  /opt/traffic-light/current/scripts/offline_runtime_probe.py \
  /opt/traffic-light/current/test_data/offline.jpg
```

离线探针解码图片、构造 32×32 的 TensorRT identity 网络并通过 CUDA 指针执行，验证结果与输入一致。它不使用交通灯模型，不能替代 P3/P4 的识别验证。模型 manifest 缺失时环境自检给出 WARN；P3 必须补齐。

此环境没有 systemd 自启服务、视频/结果监听或录制进程。视频身份认证、源帧封装、录制配额和生产服务约束分别在后续 P5–P7 实现并加入生产自检；P2 的端口检查只验证监听端口可绑定。

2026-10-29 已在独立候选目录完成 `2025_7_24.pt` 的 ONNX / TRT 10.7 FP16 构建、manifest 和 100 次连续推理；新模型联合自检 9 项 PASS，真实灯色准确率仍待金标验收。构建说明见 [MODEL_BUILD.md](MODEL_BUILD.md)。`current` 仍为 P2 release。

回退时先停止后续业务服务，将 `current` 指回先前 release；保留系统 NVIDIA 包。本次是首次部署，没有先前 release。撤销 GPU 访问时移除本规则、reload，并对规则中设备执行 `setfacl -x u:trafficlight DEVICE`；不删除其他用户 ACL，也不递归删除共享目录。
