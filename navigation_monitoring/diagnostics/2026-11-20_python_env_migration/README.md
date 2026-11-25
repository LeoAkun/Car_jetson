# 2026-11-20 Python 环境隔离与恢复记录

已按以下顺序完成：

1. 保存公共用户 Python 包和关键导入路径快照。
2. 使用 `/usr/bin/python3` 3.10.12 创建 `/home/akun/workspace/my-driving/.venv`，`include-system-site-packages=false`，用户 site-packages 不进入该环境。
3. 在 `.venv` 中固定安装 NumPy 2.2.6、OpenCV Python 5.1.0.95，并以 editable 方式安装 `rd-core`、`rd-quic`、`rd-webrtc`、`rd-vehicle` 及其依赖。
4. `.venv/bin/python -m pip check` 通过；`rd-vehicle --dump-config` 通过；`configs/vehicle.lan.yaml --check-config` 通过且未打开设备。
5. 从公共用户目录卸载 NumPy 2.2.6 和 OpenCV Python 5.1.0.95；未卸载 `paho-mqtt 1.6.1`。
6. 公共 ROS Python 恢复为系统 NumPy 1.21.5、OpenCV 4.5.4；SciPy、Matplotlib、`tf_transformations`、`transforms3d`、`cv_bridge`、OpenCV 和 MQTT 均导入成功，`pip check` 通过。
7. `/home/akun/workspace/my-driving/start.sh` 改为固定调用项目 `.venv/bin/python`，避免再次使用公共 Python。
8. 没有设置全局 `PYTHONNOUSERSITE=1`，没有启动导航、传感器或远程驾驶设备。

文件说明：

- `before_user_pip_freeze.txt`、`before_key_packages.txt`、`before_import_paths.txt`：处理前快照。
- `after_user_pip_freeze.txt`、`after_key_packages.txt`、`after_ros_import_paths.txt`：处理后公共环境快照。
- `my_driving_venv_import_paths.txt`：隔离环境关键包路径。
