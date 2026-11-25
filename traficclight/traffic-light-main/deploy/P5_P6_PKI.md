# P5/P6 设备证书签发与激活

## 当前状态

2026-11-16 已在密钥实际使用设备上生成三把 P-256 私钥及 CSR：

| 身份 | 私钥位置 | CSR subject | 证书要求 |
|---|---|---|---|
| `.150` service | `/opt/traffic-light/shared/tls/pending-20261116/service.key` | `CN=traffic-light-service-150` | `serverAuth`；DNS `traffic-light.service`；IP `192.168.1.150`；URI role `service` |
| `.140` video sender | `/home/akun/.config/traffic-light/tls/video-sender/video-sender.key` | `CN=traffic-light-video-sender-140` | `clientAuth`；URI role `video-sender` |
| `.140` controller | `/home/akun/.config/traffic-light/tls/controller/controller.key` | `CN=traffic-light-controller-140` | `clientAuth`；URI role `controller` |

私钥权限均为 `0600`，没有跨主机传输。三个 CSR、公钥 hash、固定签发脚本和策略
清单位于：

```text
/home/akun/workspace/log/trafficlight_p5_p6_2026-11-16_phase3_pki/offline-signing-request/
```

该目录不含私钥，可以复制到受控离线 CA。签名返回包只能包含 CA 证书、三个叶证书
及其校验清单。

## 离线 CA 操作

在与车辆和 Jetson 分离的签名环境中：

```bash
sudo install -d -m 0700 /secure/traffic-light-issued
./sign_tls_csrs_offline.sh \
  /secure/ca/traffic-light-ca.crt \
  /secure/ca/traffic-light-ca.key \
  /path/to/offline-signing-request/csrs \
  /secure/traffic-light-issued \
  365
```

签发脚本执行以下限制：

- CA 私钥必须为普通文件且权限为 `0400/0600`；
- CSR subject 必须与三个固定设备身份完全匹配；
- 不复制 CSR 中任意扩展，而是按角色重新写入固定 EKU、SAN 和 key usage；
- 叶证书有效期只能为 1～825 天；
- 每张证书必须通过 CA、用途和 CSR 公钥匹配检查；
- 已存在输出证书时拒绝覆盖。

CA 私钥、CA 数据库和 serial 主记录留在离线环境。车辆只接收：

```text
ca.crt
service.crt
video-sender.crt
controller.crt
```

## 返回车辆后的验证

部署前逐项检查：

```bash
openssl verify -CAfile ca.crt -purpose sslserver service.crt
openssl verify -CAfile ca.crt -purpose sslclient video-sender.crt controller.crt
openssl x509 -in service.crt -noout -checkhost traffic-light.service
openssl x509 -in service.crt -noout -checkip 192.168.1.150
```

还必须比较证书和设备私钥的公钥 SHA-256。三组 hash 应分别等于当前 CSR 清单中的
值。验证完成后，`.150` 的公有证书由 root 安装到
`/opt/traffic-light/shared/tls/`，所有者设为 `trafficlight:trafficlight`；`.140`
的证书安装到 `/home/akun/.config/traffic-light/tls/`。私钥保持 `0600`，目录保持
`0700`。

## 已完成的跨板验证

正式 CA 尚未提供，因此 2026-11-16 使用一次性验证 CA 对同一批 CSR 做了真实
`.140 → .150` 测试。验证 CA 私钥位于 `/dev/shm`，签发完成后立即销毁，未进入
仓库、设备证书目录或证据目录。验证证书不能进入 active 配置。

实际矩阵为 9/9 PASS：

- 视频 5000：TLS 1.3 mTLS、`video-sender → service` 成功；
- 结果 12345：TLS 1.3 mTLS、`controller → service` 成功；
- 错角色、客户端错 CA、服务端错 CA、错 SAN、过期服务证书、无客户端证书、
  TLS 1.2 均在业务协议前被拒绝。

正例协商 `TLS_AES_256_GCM_SHA384`。测试结束后 `.150` 临时目录、监听器和验证
证书全部删除，5000/12345 恢复无监听，P2 `current` 未改变。

同一组公开验证证书随后又用于一次限定时长的 TLV1 replay 数据面短测，设备私钥仍留在各自设备：`.140` 发送 30 帧，`.150` 接收 30 帧，关联摘要一致。服务只从 `/tmp` 启动，完成后删除。该结果只证明待正式签发的同一批设备公钥能够运行当前 replay 数据面，验证证书仍不得进入 active 配置。

证据位于：

```text
/home/akun/workspace/log/trafficlight_p5_p6_2026-11-16_phase3_pki/
```
