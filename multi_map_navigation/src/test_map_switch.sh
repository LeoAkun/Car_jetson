#!/bin/bash
# 测试地图切换的启动脚本

echo "========================================="
echo "启动地图切换测试环境"
echo "========================================="

# 1. 启动模拟TF发布节点（模拟机器人运动）
echo "1. 启动模拟TF发布节点..."
gnome-terminal -- bash -c "cd /home/akun/workspace/Car_jetson/multi_map_navigation && \
    ros2 run multi_map_navigation mock_tf_publisher; exec bash" &
sleep 2

# 2. 启动导航管理器（使用修改后的代码）
echo "2. 启动导航管理器..."
gnome-terminal -- bash -c "cd /home/akun/workspace/Car_jetson/multi_map_navigation && \
    ros2 run multi_map_navigation navigation_manager; exec bash" &
sleep 2

# 3. 等待用户输入航点列表
echo "========================================="
echo "环境已启动！"
echo ""
echo "现在可以发布航点列表来测试地图切换："
echo ""
echo "示例命令："
echo "ros2 topic pub --once /waypoint_list multi_map_navigation_msgs/msg/WaypointList '{...}'"
echo ""
echo "========================================="
