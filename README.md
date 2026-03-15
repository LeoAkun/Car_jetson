# 目录结构
① ./multi_map_navigation : 工程项目总启动文件
② ./nav2 : 包含navigation2导航的所有内容
③ ./sumulation_ws : 包含仿真的内容
④ slam : 包含SLAM的所有内容
⑤ URDF_ws ： 仿真机器人的模型文件
⑥ utils : 其他工具，如重定位、pcd点云地图转pgm栅格地图、记录gps与map坐标系等
# 启动顺序
传感器 --> re_locaization服务 -> nav2_init_pose -> slam -> navigation2
