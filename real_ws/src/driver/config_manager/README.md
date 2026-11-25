# real_ws 配置管理工具

这是一个独立 Python Web 工具，用来执行工作区编译，并修改 `src/driver` 下主要驱动配置。

不包含 `camera` 和 Livox 3D 雷达参数。

## 启动

```bash
cd /home/cplus/Desktop/Car_jetson/real_ws
python3 src/driver/config_manager/config_web.py
```

浏览器打开：

```text
http://127.0.0.1:8765
```

## 功能

- 编译安装：按项目流程执行 `livox_ros_driver2`、`serial` 预编译，再执行完整 `colcon build`。
- 配置编辑：页面保存后直接修改源码区配置文件。
- 已覆盖目录：`car_control`、`imu`、`rslidar`、`scan_laser`、`gps`、`lock_controller`、`alarm_controller`。

## 环境变量

```bash
CONFIG_MANAGER_HOST=0.0.0.0 CONFIG_MANAGER_PORT=8765 python3 src/driver/config_manager/config_web.py
```
