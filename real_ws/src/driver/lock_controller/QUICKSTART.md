# Lock Controller 快速使用指南

## 快速开始

### 1. 编译包

```bash
cd ~/yunle_sensors
colcon build --packages-select lock_controller
source install/setup.bash
```

### 2. 配置串口

编辑配置文件：
```bash
nano src/lock_controller/config/lock_controller.yaml
```

修改串口和板地址：
```yaml
lock_controller:
  ros__parameters:
    serial_port: "/dev/ttyUSB3"  # 修改为你的串口
    baud_rate: 9600
    board_address: 1             # 修改为你的板地址
```

设置串口权限：
```bash
sudo chmod 666 /dev/ttyUSB3
```

### 3. 启动节点

```bash
ros2 launch lock_controller lock_controller.launch.py
```

## 常用命令

### 打开锁

```bash
# 打开1号板的1号锁
ros2 service call /unlock lock_controller/srv/UnlockCommand "{board_address: 1, lock_address: 1}"

# 打开1号板的所有锁
ros2 service call /unlock lock_controller/srv/UnlockCommand "{board_address: 1, lock_address: 0}"
```

### 查询锁状态

```bash
# 查询1号板1号锁状态
ros2 service call /query_status lock_controller/srv/QueryLockStatus "{board_address: 1, lock_address: 1}"

# 查询1号板所有锁状态
ros2 service call /query_status lock_controller/srv/QueryLockStatus "{board_address: 1, lock_address: 0}"
```

### 使用Python示例

```bash
# 打开锁（板1，锁1）
ros2 run lock_controller unlock_example.py 1 1

# 查询状态（板1，锁1）
ros2 run lock_controller query_status_example.py 1 1

# 监听锁状态变化
ros2 run lock_controller lock_status_monitor.py
```

## 测试步骤

1. **测试连接**
   ```bash
   # 查询当前状态（应该有响应）
   ros2 service call /query_status lock_controller/srv/QueryLockStatus "{board_address: 1, lock_address: 1}"
   ```

2. **测试单锁开锁**
   ```bash
   ros2 service call /unlock lock_controller/srv/UnlockCommand "{board_address: 1, lock_address: 1}"
   ```

3. **测试全开**
   ```bash
   ros2 service call /unlock lock_controller/srv/UnlockCommand "{board_address: 1, lock_address: 0}"
   ```

4. **监听状态变化**（另开一个终端）
   ```bash
   ros2 topic echo /lock_status
   ```

## Topics列表

```bash
# 查看所有topic
ros2 topic list

# 监听锁状态
ros2 topic echo /lock_status

# 监听所有锁状态
ros2 topic echo /all_lock_status
```

## Services列表

```bash
# 查看所有service
ros2 service list

# 查看service类型
ros2 service type /unlock
ros2 service type /query_status
ros2 service type /unlock_multiple
ros2 service type /long_power_control
```

## 故障排除

### 串口权限问题
```bash
# 临时解决
sudo chmod 666 /dev/ttyUSB3

# 永久解决
sudo usermod -aG dialout $USER
# 然后重新登录
```

### 查看日志
```bash
ros2 run lock_controller lock_controller_node --ros-args --log-level debug
```

### 检查串口
```bash
# 列出所有串口
ls -l /dev/ttyUSB*

# 检查串口是否被占用
lsof /dev/ttyUSB3
```

## 硬件检查清单

- [ ] RS485连接正确（A-A, B-B, GND）
- [ ] 电源供电正常（12-24V DC）
- [ ] 板地址拨码开关设置正确
- [ ] 锁的反馈线连接正常
- [ ] 串口设备路径正确
- [ ] 串口权限已设置

## 进阶功能

### 批量开锁

创建一个24位布尔数组，true的位置表示要打开的锁：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lock_controller.srv import UnlockMultiple

def unlock_multiple_locks():
    rclpy.init()
    node = Node('unlock_multiple_test')
    client = node.create_client(UnlockMultiple, 'unlock_multiple')

    while not client.wait_for_service(timeout_sec=1.0):
        print('等待服务...')

    request = UnlockMultiple.Request()
    request.board_address = 1

    # 创建24位掩码，打开第2、10、18号锁
    mask = [False] * 24
    mask[1] = True   # 锁2
    mask[9] = True   # 锁10
    mask[17] = True  # 锁18
    request.lock_mask = mask

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result():
        print(f'结果: {future.result().success}')
        print(f'消息: {future.result().message}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    unlock_multiple_locks()
```

### 长通电控制（电磁锁）

```bash
# 开启长通电
ros2 service call /long_power_control lock_controller/srv/LongPowerControl "{board_address: 1, lock_address: 1, enable: true}"

# 关闭长通电
ros2 service call /long_power_control lock_controller/srv/LongPowerControl "{board_address: 1, lock_address: 1, enable: false}"
```

## 技术支持

如有问题，请查看：
- README.md 完整文档
- 485锁控板通讯协议2024A1.pdf 协议文档
- 或联系 cplus <3174680732@qq.com>
