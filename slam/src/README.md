# 一、功能描述
- slam建图功能包，负责发布odom->base_link的tf变换
# 二、目录结构
- LIO-SAM：Liosam算法
- pcd_global: 建图完成后的pcd地图文件
# 三、建图
## (1)
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
ros2 run record_gps_map record_gps_map 
```

## (4)保存pcd和csv
- 注意随时保存，免得主机重启
- 保存pcd，保存路径查看/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/config/params_rslidar.yaml内的savePCDDirectory字段下
```shell
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap
```

- 保存csv，默认路径查看源码硬编码为/home/akun/workspace/Car_jetson/utils/src/record_gps_map/record_gps_map.csv
```shell
ctrl + c
```