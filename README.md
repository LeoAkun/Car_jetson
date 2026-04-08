# 一、目录结构
- ① ./multi_map_navigation : 工程项目总启动文件
- ② ./nav2 : 包含navigation2导航的所有内容
- ③ ./sumulation_ws : 包含仿真的内容
- ④ slam : 包含SLAM的所有内容
- ⑤ URDF_ws ： 仿真机器人的模型文件
- ⑥ utils : 其他工具，如重定位、pcd点云地图转pgm栅格地图、记录gps与map坐标系等
# 二、项目启动前需要的准备
- ① pcd地图: 用于获取pgm导航地图与重定位算法(使用slam获取)
- ② map2slam: csv文件，用于将gps坐标转为局部坐标系(在utils目录下的record_gps_map包，边建图边运行)
- ③ pgm地图: 用于加载navigation2全局代价地图(在utils目录下的pcd2pgm包，转换时耐心等待)
- ④ 传感器是否全部启动成功: 包括imu、gps、两个单线雷达与、激光雷达、底盘驱动
# 三、使用rviz2点击导航启动顺序
传感器 --> re_locaization服务 -> nav2_init_pose -> slam -> navigation2
# 四、直接启动项目
```shell
ros2 launch multi_map_navigation multi_map_navigation
```
# 五、tf坐标系说明
## 1.坐标系
|坐标系|说明|
|:--:|:--:|
|map|以地图起点为原点，即建图时的起始点为原点|
|odom|以机器人的启动位置为起点|
|base_link|以机器人物理中心为原点|
|传感器_link|以传感器为原点|

## 2.tf变换
|tf变换|说明|
|:--:|:--:|
|map-->odom|由重定位nav2_init_pose发布, 计算机器人在地图的哪个位置启动|
|odom-->base_link|由slam发布, slam计算相对启动位置移动的距离|
|base_link-->传感器_link|由驱动发布, 各传感器与base_link的位置关系|

# 六、传感器相关
## 1.传感器说明
- 1.单线雷达：连接到交换机， 192.168.1.12  192.168.1.13
- 2.CAN盒：连接到交换机 192.168.1.10
- 3.多线雷达：连接到主机 
- 4.jetson计算板：连接到交换机 192.168.1.150  192.168.1.160 用户:jetson 密码:yahboom
- 5.路由器：连接到交换机
- 6.主机: 连接到交换机
- 7.GPS、IMU、拓展坞等：通过usb连接到主机
## 2.开机自启服务
- 关闭开机自启传感器服务
```
sudo systemctl kill robot-service.service
sudo systemctl stop robot-service.service
```