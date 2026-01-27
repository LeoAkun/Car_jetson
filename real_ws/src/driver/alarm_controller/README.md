# 设备名
- /dev/ttyUSB1

# 调用示例

- 开启警报
```shell
ros2 service call /control_alarm alarm_interface/srv/Alarm "{operate: true}"
```

- 关闭警报
```shell
ros2 service call /control_alarm alarm_interface/srv/Alarm "{operate: false}"
```
