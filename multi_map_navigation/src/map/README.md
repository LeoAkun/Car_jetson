# 一、功能

- 保存地图信息文件，包括：原始pcd地图(xxx_raw.pcd)、清除人影和地面的pcd地图(xxx_clean.pcd)、清除人影和地面的pgm地图及其配置文件(xxx_clean.pgm、xxx_clean.yaml)、地图的gps与地图坐标系对应csv表格(xxx.csv)

# 二、转pgm/yaml流程

- 从电脑上传图实例（windows终端）：

```bash
scp -r "C:\Users\25715\OneDrive\Desktop\文档\*" akun@192.168.53.104:/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/redtest/

# 本机路径+小车IP+目标路径
```

- `pcd`转`pgm`:

1. 编辑这个文件：

```bash
/home/akun/workspace/Car_jetson/utils/src/pcd2pgm/config/pcd2pgm.yaml
# 把 pcd_file 改成你的 .pcd 文件路径，比如：
pcd2pgm:
  ros__parameters:
    pcd_file: /home/akun/workspace/Car_jetson/xxx/your_map.pcd

```

1. 启动文件里会读取参数文件并启动 `pcd2pgm_node`：

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
# 如果想指定别的参数文件，也可以这样：
ros2 launch pcd2pgm pcd2pgm_launch.py params_file:=/home/akun/workspace/Car_jetson/utils/src/pcd2pgm/config/pcd2pgm.yaml
```

1. 另开一个终端保存成 `.pgm`

因为这个节点会发布 `map`话题，所以再开一个终端执行：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/akun/workspace/Car_jetson/multi_map_navigation/src/map/redtest/redtest_clean
# 目标路径自己修改
```

# 三、单独启动nav2手动发送目标导航流程

1. 第一步：把静态 `map -> odom` 注释掉
2. 第二步：重新构建 / source / 重启 SLAM

```bash
ros2 launch lio_sam run.real_launch.py
```

1. 第三步：检查的 `map -> odom`

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

1. 第四步：启动nav2

```bash
ros2 launch nav2 run.real_launch.py 
map:=/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/test_map/testmap_clean.yaml
# 改成自己的路径
```

