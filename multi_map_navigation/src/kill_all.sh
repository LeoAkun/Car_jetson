#!/usr/bin/env bash
# kill_all.sh
# 用于结束 re_localization、nav2_init_pose、lio_sam、nav2 相关的所有进程
# 使用方法： chmod +x kill_all.sh && ./kill_all.sh

set -euo pipefail

echo "=== 开始结束 ROS 相关进程 ==="

# 1. 先尝试优雅终止（SIGTERM）
echo "发送 SIGTERM 信号（优雅关闭）..."

# re_localization 相关（launch + pcl_localization_node）
pkill -f "ros2 launch re_localization run.real_launch.py" || true
pkill -f "pcl_localization_node" || true

# nav2_init_pose 相关（run + executable）
pkill -f "ros2 run nav2_init_pose nav2_init_pose" || true
pkill -f "nav2_init_pose/lib/nav2_init_pose/nav2_init_pose" || true

# lio_sam 相关（launch + 所有子节点）
pkill -f "ros2 launch lio_sam run.real_launch.py" || true
pkill -f "lio_sam_imuPreintegration" || true
pkill -f "lio_sam_imageProjection" || true
pkill -f "lio_sam_featureExtraction" || true
pkill -f "lio_sam_mapOptimization" || true
pkill -f "static_transform_publisher.*map odom" || true
pkill -f "navsat_transform_node" || true
pkill -f "ekf_node" || true

# nav2 相关（container、rviz2、costmap 等）
pkill -f "component_container_isolated.*nav2_container" || true
pkill -f "rviz2.*nav2_default_view.rviz" || true
pkill -f "rviz2.*lio_sam/config/rviz2.rviz" || true
pkill -f "nav2_costmap_2d_cloud" || true

echo "等待 5 秒，让进程优雅退出..."
sleep 5

# 2. 检查是否还有残留进程，并强制杀死
echo ""
echo "检查残留进程并强制杀死（SIGKILL）..."

# 再次列出可能残留的进程
echo "当前残留的进程（如果有）："
ps -ef | grep -E "re_localization|nav2_init_pose|lio_sam|nav2_container|rviz2.*nav2|rviz2.*lio_sam|pcl_localization|costmap_2d_cloud|ekf_node" | grep -v grep || echo "无残留进程"

# 强制杀死残留
pkill -9 -f "ros2 launch re_localization" || true
pkill -9 -f "pcl_localization_node" || true
pkill -9 -f "nav2_init_pose" || true
pkill -9 -f "lio_sam_" || true
pkill -9 -f "component_container_isolated.*nav2" || true
pkill -9 -f "rviz2.*nav2" || true
pkill -9 -f "rviz2.*lio_sam" || true
pkill -9 -f "nav2_costmap_2d_cloud" || true

echo ""
echo "等待 3 秒后最终检查..."
sleep 3

# 3. 最终确认
echo "最终进程状态："
ps -ef | grep -E "re_localization|nav2_init_pose|lio_sam|nav2_container|rviz2.*nav2|rviz2.*lio_sam|pcl_localization|costmap_2d_cloud" | grep -v grep || echo "所有目标进程已结束"

echo ""
echo "=== 结束完成 ==="
echo "如果还有残留，请手动检查："
echo "  ps -ef | grep -E 're_localization|lio_sam|nav2|rviz2'"
