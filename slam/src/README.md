# 一、功能描述

- slam建图功能包，负责发布odom->base_link的tf变换

# 二、目录结构

- LIO-SAM：Liosam算法
- pcd_global: 建图完成后的pcd地图文件

# 三、建图流程

## (1)打开liosam启动文件的注释

- 把/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/launch/run.real_launch.py文件下的下面的代码，打开注释

```python
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
             parameters=[parameter_file],
             output='screen'
             ), 
```

## (2)启动建图

```shell
ros2 launch lio_sam run.real_launch.py
```

## (3)记录csv文件

- 源码路径:/home/akun/workspace/Car_jetson/utils/src/record_gps_map/record_gps_map

```shell
# 使用该命令，会一直自动保存csv文件，默认路径为/home/akun/workspace/Car_jetson/utils/src/record_gps_map/record_gps_map.csv，每次都会覆盖record_gps_map.csv
ros2 run record_gps_map record_gps_map 

# 建图结束后：
ctrl + c
```

## (4)保存pcd

- 注意随时保存，免得主机重启
- 保存pcd，保存路径查看/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/config/params_rslidar.yaml内的savePCDDirectory字段下

```shell
# 需要手动使用以下命令，调用服务保存pcd
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap
# 默认保存路径：/workspace/Car_jetson/slam/src/pcd_global/test/
# 每次调用保存服务，都会自动生成：
CornerMap.pcd           # 角点特征地图
SurfMap.pcd             # 平面特征地图
GlobalMap.pcd           # 完整全局点云地图，需要挖路（由前两者合成）
trajectory.pcd          # 机器人建图轨迹
transformations.pcd     # 关键帧位姿变换
```

