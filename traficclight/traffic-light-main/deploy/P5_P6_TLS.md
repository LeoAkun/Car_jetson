# P5/P6 TLS 1.3 双向认证约定

本约定对应 `traffic_light_stream/tls.py`。它只建立和认证一条已经连接的 socket，不创建监听器、不连接业务端口、不创建控制 session，也不改变候选模型的控制资格。

## 1. 认证目标

P5/P6 的视频和结果连接都要求 TLS 1.3 mTLS：

| 连接 | 服务端证书 | 客户端证书 | 服务端授权的客户端角色 |
|---|---|---|---|
| `.140` → `.150:5000` 视频 | `service` | `video-sender` | `video-sender` |
| `.140` → `.150:12345` 结果/控制 | `service` | `controller` | `controller` |

“证书由同一 CA 签发”只证明签发链，不代表应用授权。代码在 OpenSSL 完成 CA、有效期、主机名/SAN 和 EKU 校验后，还要求证书 URI SAN 中恰好有一个下列角色：

```text
urn:car-jetson:traffic-light:role:service
urn:car-jetson:traffic-light:role:video-sender
urn:car-jetson:traffic-light:role:controller
```

角色缺失、未知、重复或与端点策略不一致时，连接立即关闭。视频 sender 证书不能用于控制连接，controller 证书也不能用于视频连接。

## 2. 证书要求

- 服务端证书：EKU 包含 `serverAuth`；SAN 包含客户端实际使用的 DNS 名或 IP 地址；URI SAN 为 `service`。
- 视频客户端证书：EKU 包含 `clientAuth`；URI SAN 为 `video-sender`。
- 结果客户端证书：EKU 包含 `clientAuth`；URI SAN 为 `controller`。
- CA 私钥不放入车辆、Jetson、release、仓库或测试报告。
- 每个角色使用独立私钥。私钥文件必须是普通文件、绝对路径，权限只能是 `0400` 或 `0600`。
- 正式证书必须在两台主机 wall clock 可信后签发和安装。代码不会关闭证书有效期或主机名验证，也不会回退到明文。

测试使用 `tempfile` 目录和临时 CA；测试退出后目录自动删除。仓库中的四个 JSON 只含预期安装路径和策略，不含证书或私钥。

## 3. 严格配置

每个端点单独加载一个六字段 JSON：

| 字段 | 含义 |
|---|---|
| `ca_file` | 受信 CA 证书的绝对路径 |
| `cert_file` | 本端证书链的绝对路径 |
| `key_file` | 本端私钥的绝对路径 |
| `server_name` | 客户端验证的 DNS/IP；服务端必须为 `null` |
| `expected_peer_role` | 该端点允许的唯一对端角色 |
| `handshake_timeout_s` | 从开始握手到认证结束的总时限，范围 0.05–30 秒 |

解析器拒绝未知字段、缺失字段、重复 JSON key、NaN/Infinity、相对路径、未知角色、布尔型伪装数值和越界时限。TLS context 创建时再检查文件存在、文件类型和私钥权限，因此配置文件通过解析不代表凭据已经可用。

示例：

- `config/tls_video_sender.example.json`
- `config/tls_video_service.example.json`
- `config/tls_result_controller.example.json`
- `config/tls_result_service.example.json`

## 4. 握手运行过程

1. 上层只建立 TCP socket 或接受一个 TCP socket；此时不能读取 TLV1/NDJSON，也不能进入 ready。
2. client context 固定 `PROTOCOL_TLS_CLIENT`、`CERT_REQUIRED`、hostname check 和 TLS 1.3 上下界；server context 固定 `PROTOCOL_TLS_SERVER`、`CERT_REQUIRED` 和 TLS 1.3 上下界。
3. socket 进入非阻塞握手。所有 `WANT_READ/WANT_WRITE` 共用一个 monotonic 绝对截止点；收到部分握手数据不会延长时限。
4. OpenSSL 验证 CA、证书有效期、EKU 和客户端配置的 DNS/IP SAN。
5. 应用读取已验证证书的 URI SAN，并验证该端点的角色策略。
6. 只有上述步骤全部通过，wrapper 才返回 `SSLSocket`。上层此后才允许设置 transport ready、解析 TLV1/NDJSON 或创建 session。
7. 配置、包装、握手、证书或角色任一失败都会关闭 socket；没有明文 fallback。

## 5. 已覆盖的 loopback 负例

`tests/test_tls.py` 只绑定 `127.0.0.1`，当前覆盖：

- 有效 DNS SAN 和有效 IP SAN 的 TLS 1.3 mTLS；
- 客户端无证书；
- 客户端不信任服务端 CA；
- 服务端不信任客户端 CA；
- 服务端 SAN 不匹配；
- 过期服务端证书；
- CA 正确但应用角色错误；
- 客户端 EKU 错误；
- TLS 1.2-only 降级；
- 握手只发起但不继续，达到总 deadline；
- 握手期间断开；
- 不安全私钥权限、相对路径和严格配置错误；
- context 创建失败时原始 socket 也被关闭。

这些测试证明认证 helper 的 fail-closed 行为；它们不证明两台设备时钟、正式证书、业务端口、session、replay/相机链路或 TensorRT 实时服务已经完成。

## 6. 正式部署前检查

正式设备证书进入 staging 前必须同时满足：

1. `.140` 和 `.150` 的 `timedatectl show -p NTPSynchronized` 均为 `yes`，并保留重启后证据；
2. 证书 SAN 与实际连接名/IP 一致，EKU 和角色 URI 正确；
3. 私钥位于 release 之外，只由运行账户读取，权限 `0400/0600`；
4. staging self-check 能读取配置、创建 context，但不启动业务 listener；
5. 先执行 loopback 和两板限定时长测试，再考虑 systemd 或 `current` 切换。

2026-11-16 已由 `.140` 的受限 chrony 服务完成 `.150/.160` 授时，并通过逐台重启验收，wall-clock Gate 已解除。同日已在各自设备生成 service、video-sender、controller 私钥和 CSR；私钥没有跨设备传输。由于当前环境没有外部正式 CA，正式 CSR 仍待离线签名。

为验证设备路径，使用签发后立即销毁私钥的一次性验证 CA 完成了真实 `.140 → .150` 矩阵：视频 5000、结果 12345 两条 TLS 1.3 mTLS 正例通过；错角色、双向错 CA、错 SAN、过期、无客户端证书和 TLS 1.2 七类负例全部在业务协议前被拒绝。测试结束后 `.150` 临时证书和监听器已删除，`current` 仍为 P2。正式签发、返回包检查和激活流程见 `deploy/P5_P6_PKI.md`。

同日又使用这些非生产验证证书完成一次真实 TLV1 replay 数据面短测：`.140` 以 10 Hz 向 `.150:5000` 发送 30 个 1280×720 JPEG，`.150` 接收 30/30，双方帧序号/JPEG 关联摘要一致。该测试证明 TLS helper 之后的业务帧传输路径可用；它不替代正式 CA Gate、长期重连、30 分钟稳定性、真实相机或实时推理验收。
