# MQTT 原始消息监听节点

`mqtt_raw_monitor` 独立订阅服务器下发的任务消息，显示 MQTT topic、接收时间、
QoS、retain、dup、原始字节数及完整 payload。它不创建任务发布器，不发送 MQTT
业务消息，也不启动导航。默认只手动运行，不随导航 launch 自动启动。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
source /home/akun/workspace/Car_jetson/real_ws/install/setup.bash
cd /home/akun/workspace/Car_jetson/multi_map_navigation
colcon build --packages-select multi_map_navigation --symlink-install
source install/setup.bash
ros2 run multi_map_navigation mqtt_raw_monitor
```

默认直接读取当前安装包的 `config/mqtt_config.yaml`，使用
`mqtt_task_receiver.ros__parameters` 中的服务器、端口、账号、密码、task_topic、
qos、reconnect_delay。当前任务 topic 是 `robot/LS1234567890/down/cmd/sendTask`。
不需要复制账号密码，也不要把原任务接收器的参数文件直接作为此节点的 ROS 参数传入。

看到“订阅就绪”才表示服务器已确认订阅；没有消息时会继续等待。
连接失败或断开后自动重试，重连后自动重新订阅。按 Ctrl+C 退出。
节点会显示保留消息标记，历史 retained 消息不一定是刚下发的任务。

每次启动默认生成独立的随机 Client ID，可和任务接收器同时订阅同一个普通 topic。
若另一个任务接收器也在运行，它仍会执行自己的任务处理逻辑；要只观察不执行，
应只启动本监听节点。节点拒绝使用配置里已有接收/上报节点的 Client ID，
也拒绝 `$share/`、`$queue/` 共享订阅，避免分流其他客户端的任务。

## 可选参数

```bash
# 在原文之后附加格式化 JSON 预览
ros2 run multi_map_navigation mqtt_raw_monitor --ros-args -p pretty_print:=true

# 保存原始消息为 JSONL，单个文件默认约 10 MiB，保留 3 个轮转备份
ros2 run multi_map_navigation mqtt_raw_monitor --ros-args \
  -p log_file:=/home/akun/workspace/mqtt_raw_messages.jsonl

# 指定其他连接配置文件，格式仍为 mqtt_task_receiver.ros__parameters
ros2 run multi_map_navigation mqtt_raw_monitor --ros-args \
  -p config_file:=/absolute/path/mqtt_config.yaml

# 临时监听其他普通 topic，支持 MQTT 通配符
ros2 run multi_map_navigation mqtt_raw_monitor --ros-args \
  -p topic:=robot/LS1234567890/up/ack
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `config_file` | 安装包内 mqtt_config.yaml | 复用现有连接配置 |
| `topic` | 空，使用 task_topic | 监听的普通主题过滤器 |
| `client_id` | 空，启动时随机生成 | 可选的独立 MQTT Client ID |
| `pretty_print` | false | 追加格式化预览，原文仍保留 |
| `log_file` | 空 | 为空时不额外保存 JSONL 文件 |
| `log_max_bytes` | 10485760 | 触发日志轮转的文件大小阈值 |
| `log_backup_count` | 3 | 最多保留的轮转文件数量 |

JSONL 每行是一条消息，包含 `received_at`（UTC 时间）、`topic`、`qos`、
`retained`、`duplicate`、`payload_bytes`、`payload_text`、`payload_base64`、
`json_valid`。`payload_text` 保留原始 UTF-8 文本，包括空格和换行，
不是重新序列化的 JSON。`payload_base64` 可还原精确原始字节。
JSON 格式错误也会显示并保存；非 UTF-8 内容会记录 Base64 和解码错误。
终端中含 NUL 的文本会以转义形式展示，JSONL 原文不变；嵌套过深、无法解析的
JSON 同样保留原始消息，不影响后续接收。
JSONL 按文件大小轮转，单条超过阈值的大消息仍会完整保存。
原始消息可能包含业务数据，请按需选择日志路径与保留大小。

本节点只能确认本客户端收到服务器下行消息；确认后端是否收到状态上报，
仍应查看后端日志或订阅对应的上行 topic。
