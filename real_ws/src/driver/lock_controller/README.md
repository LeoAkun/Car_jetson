# Lock Controller ROS2 Package

## 概述

这是一个用于控制485锁控板的ROS2功能包。该包实现了完整的RS485通讯协议，支持广州硕铭科技有限公司的锁控板（485通讯系列）。

## 硬件要求

- **通讯方式**: RS485多机通讯
- **波特率**: 9600
- **数据位**: 8
- **停止位**: 1
- **流控**: None
- **工作电压**: 12-24V DC
- **支持设备**: 最多32块锁控板串联使用，每块板最多支持24路锁

## 功能特性

本包实现了锁控板通讯协议中的所有功能：

1. **单锁开锁** - 打开指定板子上的指定锁
2. **全开命令** - 一次性打开一个板子上的所有锁
3. **查询锁状态** - 查询单个锁或所有锁的状态
4. **批量开锁** - 一条命令打开多个指定的锁
5. **长通电功能** - 用于控制电磁锁的持续通电/断电
6. **主动反馈** - 实时接收锁状态变化的主动上报

## 安装

### 依赖

确保已安装ROS2（建议Humble或更新版本）：

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### 构建

```bash
cd ~/yunle_sensors  # 或你的工作空间路径
colcon build --packages-select lock_controller
source install/setup.bash
```

## 配置

编辑配置文件 `config/lock_controller.yaml`：

```yaml
lock_controller:
  ros__parameters:
    serial_port: "/dev/ttyUSB3"  # 串口设备路径
    baud_rate: 9600              # 波特率（固定为9600）
    board_address: 1             # 默认板地址（1-32）
```

### 串口权限设置

给予串口设备访问权限：

```bash
sudo chmod 666 /dev/ttyUSB3
# 或者将用户添加到dialout组（需要重新登录）
sudo usermod -aG dialout $USER
```

## 使用方法

### 启动节点

使用launch文件启动：

```bash
ros2 launch lock_controller lock_controller.launch.py
```

或直接运行节点：

```bash
ros2 run lock_controller lock_controller_node --ros-args --params-file ./src/lock_controller/config/lock_controller.yaml
```

## ROS2 接口

### Topics

#### 发布的Topics

- **`/lock_status`** (`lock_controller/msg/LockStatus`)
  - 发布单个锁的状态变化
  - 字段：
    - `uint8 board_address`: 板地址（1-32）
    - `uint8 lock_address`: 锁地址（1-24）
    - `bool is_locked`: 锁状态（true=已锁，false=已开）

- **`/all_lock_status`** (`lock_controller/msg/AllLockStatus`)
  - 发布一个板子上所有锁的状态
  - 字段：
    - `uint8 board_address`: 板地址（1-32）
    - `bool[] lock_states`: 所有锁的状态数组（最多24个）

### Services

#### 1. 开锁服务 `/unlock`

**类型**: `lock_controller/srv/UnlockCommand`

**功能**: 打开指定的单个锁或全部锁

**请求**:
```
uint8 board_address    # 板地址（1-32）
uint8 lock_address     # 锁地址（1-24），0表示全部锁
```

**响应**:
```
bool success           # 是否成功
string message         # 反馈消息
```

**使用示例**:

```bash
# 打开1号板的1号锁
ros2 service call /unlock lock_controller/srv/UnlockCommand "{board_address: 1, lock_address: 1}"

# 打开1号板的所有锁
ros2 service call /unlock lock_controller/srv/UnlockCommand "{board_address: 1, lock_address: 0}"
```

#### 2. 查询锁状态服务 `/query_status`

**类型**: `lock_controller/srv/QueryLockStatus`

**功能**: 查询单个锁或所有锁的状态

**请求**:
```
uint8 board_address    # 板地址（1-32）
uint8 lock_address     # 锁地址（1-24），0表示查询所有锁
```

**响应**:
```
bool success              # 是否成功
bool is_locked            # 锁状态（仅查询单个锁时有效）
bool[] all_lock_states    # 所有锁状态（lock_address=0时有效）
string message            # 反馈消息
```

**使用示例**:

```bash
# 查询1号板的1号锁状态
ros2 service call /query_status lock_controller/srv/QueryLockStatus "{board_address: 1, lock_address: 1}"

# 查询1号板的所有锁状态
ros2 service call /query_status lock_controller/srv/QueryLockStatus "{board_address: 1, lock_address: 0}"
```

#### 3. 批量开锁服务 `/unlock_multiple`

**类型**: `lock_controller/srv/UnlockMultiple`

**功能**: 一次性打开多个指定的锁

**请求**:
```
uint8 board_address    # 板地址（1-32）
bool[] lock_mask       # 锁掩码数组（最多24个元素），true表示要打开该锁
```

**响应**:
```
bool success           # 是否成功
string message         # 反馈消息
```

**使用示例**:

```bash
# 打开1号板的第2、10、18号锁
# 创建24位掩码，第2、10、18位为true
ros2 service call /unlock_multiple lock_controller/srv/UnlockMultiple "{board_address: 1, lock_mask: [false, true, false, false, false, false, false, false, false, true, false, false, false, false, false, false, false, true, false, false, false, false, false, false]}"
```

#### 4. 长通电控制服务 `/long_power_control`

**类型**: `lock_controller/srv/LongPowerControl`

**功能**: 控制电磁锁的长通电/断电（需要固件支持）

**请求**:
```
uint8 board_address    # 板地址（1-32）
uint8 lock_address     # 锁地址（1-24）
bool enable            # true=开启长通电，false=关闭长通电
```

**响应**:
```
bool success           # 是否成功
string message         # 反馈消息
```

**使用示例**:

```bash
# 开启1号板1号锁的长通电
ros2 service call /long_power_control lock_controller/srv/LongPowerControl "{board_address: 1, lock_address: 1, enable: true}"

# 关闭1号板1号锁的长通电
ros2 service call /long_power_control lock_controller/srv/LongPowerControl "{board_address: 1, lock_address: 1, enable: false}"
```

## Python客户端示例

### 开锁示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lock_controller.srv import UnlockCommand

class LockClient(Node):
    def __init__(self):
        super().__init__('lock_client')
        self.client = self.create_client(UnlockCommand, 'unlock')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务...')

    def unlock(self, board_address, lock_address):
        request = UnlockCommand.Request()
        request.board_address = board_address
        request.lock_address = lock_address

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'结果: {response.success}, {response.message}')
            return response.success
        else:
            self.get_logger().error('服务调用失败')
            return False

def main():
    rclpy.init()
    client = LockClient()

    # 打开1号板的1号锁
    client.unlock(1, 1)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 查询状态示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lock_controller.srv import QueryLockStatus

class StatusQueryClient(Node):
    def __init__(self):
        super().__init__('status_query_client')
        self.client = self.create_client(QueryLockStatus, 'query_status')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务...')

    def query_all_status(self, board_address):
        request = QueryLockStatus.Request()
        request.board_address = board_address
        request.lock_address = 0  # 0表示查询所有锁

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'板{board_address}锁状态:')
                for i, locked in enumerate(response.all_lock_states):
                    status = "已锁" if locked else "已开"
                    self.get_logger().info(f'  锁{i+1}: {status}')
            return response.success
        else:
            self.get_logger().error('服务调用失败')
            return False

def main():
    rclpy.init()
    client = StatusQueryClient()

    # 查询1号板的所有锁状态
    client.query_all_status(1)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 订阅锁状态变化

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lock_controller.msg import LockStatus

class LockStatusSubscriber(Node):
    def __init__(self):
        super().__init__('lock_status_subscriber')
        self.subscription = self.create_subscription(
            LockStatus,
            'lock_status',
            self.status_callback,
            10)

    def status_callback(self, msg):
        status = "已锁" if msg.is_locked else "已开"
        self.get_logger().info(
            f'板{msg.board_address}的锁{msg.lock_address}状态变化: {status}')

def main():
    rclpy.init()
    subscriber = LockStatusSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 通讯协议说明

### 命令格式

所有命令均为16进制格式，使用BCC/异或校验。

基本格式：`命令头 板地址 锁地址/参数 功能码 校验`

### 开锁命令

- **命令头**: `0x8A`
- **功能码**: `0x11`
- **例**: `8A 01 01 11 9B` - 打开1号板的1号锁
- **全开**: `8A 01 00 11 9A` - 打开1号板的所有锁

### 查询命令

- **命令头**: `0x80`
- **功能码**: `0x33`
- **例**: `80 01 01 33 B3` - 查询1号板1号锁状态
- **查询所有**: `80 01 00 33 B2` - 查询1号板所有锁状态

### 批量开锁命令

- **命令头**: `0x90`
- **例**: `90 01 02 02 02 93` - 打开1号板的2号、10号、18号锁

### 长通电命令

- **开启命令头**: `0x9A`
- **关闭命令头**: `0x9B`
- **例**: `9A 01 01 11 8B` - 开启1号板1号锁的长通电

### 状态反馈

- **被动反馈**: `0x8A`或`0x80`开头（响应查询或命令）
- **主动反馈**: `0x82`开头（锁状态自动上报）
- **状态值**: `0x11`=锁关闭，`0x00`=锁打开

## 故障排除

### 1. 无法打开串口

**错误信息**: `Failed to open serial port`

**解决方法**:
- 检查串口设备是否存在：`ls -l /dev/ttyUSB*`
- 检查权限：`sudo chmod 666 /dev/ttyUSB3`
- 检查是否被其他程序占用：`lsof /dev/ttyUSB3`

### 2. 通信超时

**错误信息**: `Timeout waiting for response`

**解决方法**:
- 检查RS485连接是否正常
- 确认板地址设置是否正确（检查拨码开关）
- 检查波特率是否为9600
- 确认供电是否正常（12-24V DC）

### 3. 开锁无响应

**可能原因**:
- 锁的反馈类型不匹配（上锁短路 vs 开锁短路）
- 检查反馈信号线连接
- 确认锁的工作电压

### 4. 主动反馈不工作

**注意**: 主动反馈功能并非所有产品都支持，部分产品不带此功能。

## 硬件连接

### RS485接线

- **485通讯GND**: 接地
- **485通讯A**: RS485-A
- **485通讯B**: RS485-B
- **DC12V**: 电源正极（12-24V）
- **GND**: 电源负极

### 地址开关设定

使用板上的拨码开关设置板地址（1-32），支持最多32块板串联。

## 支持的锁控板型号

- 2路锁控板
- 8路锁控板
- 12路锁控板
- 18路锁控板
- 24路锁控板
- 50路锁控板

## 技术参数

- **通讯方式**: RS485
- **波特率**: 9600bps
- **数据位**: 8
- **停止位**: 1
- **校验**: None
- **流控**: None
- **命令格式**: HEX
- **最大设备数**: 32块板（1-99台设备理论支持）
- **每板最大锁数**: 24路（部分型号支持50路）

## 许可证

MIT License

## 作者

cplus <3174680732@qq.com>

## 版本历史

- **v1.0.0** (2025) - 初始版本
  - 实现完整的485锁控板通讯协议
  - 支持所有基本功能
  - 提供ROS2服务和话题接口

## 参考资料

- 锁控板通讯协议文档: `485锁控板通讯协议2024A1.pdf`
- 制造商: 广州硕铭科技有限公司
