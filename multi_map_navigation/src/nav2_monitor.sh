#!/bin/bash
# Nav2 导航健康监控脚本
# 用于现场排查"膨胀层消失、走走停停、程序崩溃"等问题
# 同时监控：TF 延迟、传感器话题频率、CPU 占用、Nav2 节点存活状态
#
# 使用方法：
#   chmod +x nav2_monitor.sh
#   ./nav2_monitor.sh
#
# 输出会同时打印到终端和保存到 ~/nav2_monitor.log

LOG_FILE="$HOME/nav2_monitor.log"
INTERVAL=3  # 每隔 3 秒采集一次

echo "======================================" | tee -a "$LOG_FILE"
echo " Nav2 导航健康监控 - $(date '+%Y-%m-%d %H:%M:%S')" | tee -a "$LOG_FILE"
echo " 日志保存到: $LOG_FILE" | tee -a "$LOG_FILE"
echo " 采集间隔: ${INTERVAL}s, Ctrl+C 停止" | tee -a "$LOG_FILE"
echo "======================================" | tee -a "$LOG_FILE"

while true; do
    echo "" | tee -a "$LOG_FILE"
    echo "--- $(date '+%H:%M:%S') ---" | tee -a "$LOG_FILE"

    # 1. TF 延迟检测: map -> base_link
    echo "[TF] map -> base_link:" | tee -a "$LOG_FILE"
    timeout 2 ros2 run tf2_ros tf2_echo map base_link --wait-for-transform 1.0 2>&1 | \
        grep -E "Translation|could not|timed out|Exception|Lookup" | head -3 | tee -a "$LOG_FILE"
    
    # 2. TF 延迟检测: odom -> base_link
    echo "[TF] odom -> base_link:" | tee -a "$LOG_FILE"
    timeout 2 ros2 run tf2_ros tf2_echo odom base_link --wait-for-transform 1.0 2>&1 | \
        grep -E "Translation|could not|timed out|Exception|Lookup" | head -3 | tee -a "$LOG_FILE"

    # 3. 激光雷达话题频率
    echo "[SCAN] /laser/scan_merge_laser 频率:" | tee -a "$LOG_FILE"
    timeout 3 ros2 topic hz /laser/scan_merge_laser --window 5 2>&1 | \
        grep -E "average rate|no new messages" | head -2 | tee -a "$LOG_FILE"

    # 4. cmd_vel 频率（判断 controller 是否在正常输出）
    echo "[CMD_VEL] /cmd_vel 频率:" | tee -a "$LOG_FILE"
    timeout 3 ros2 topic hz /cmd_vel --window 5 2>&1 | \
        grep -E "average rate|no new messages" | head -2 | tee -a "$LOG_FILE"

    # 5. Nav2 关键节点存活检查
    echo "[NODES] Nav2 关键节点:" | tee -a "$LOG_FILE"
    NODES_TO_CHECK=("controller_server" "planner_server" "bt_navigator" "local_costmap" "global_costmap")
    ALIVE_NODES=$(ros2 node list 2>/dev/null)
    for node in "${NODES_TO_CHECK[@]}"; do
        if echo "$ALIVE_NODES" | grep -q "$node"; then
            echo "  $node: 存活" | tee -a "$LOG_FILE"
        else
            echo "  $node: !!!未找到!!!" | tee -a "$LOG_FILE"
        fi
    done

    # 6. CPU 和内存占用（前5个高 CPU 进程）
    echo "[SYSTEM] CPU/MEM TOP 5:" | tee -a "$LOG_FILE"
    ps aux --sort=-%cpu | head -6 | awk '{printf "  %-8s CPU:%s%% MEM:%s%% CMD:%s\n", $2, $3, $4, $11}' | tee -a "$LOG_FILE"

    # 7. 系统总 CPU 使用率
    echo "[SYSTEM] 总 CPU 使用率:" | tee -a "$LOG_FILE"
    top -bn1 | grep "Cpu(s)" | awk '{print "  " $0}' | tee -a "$LOG_FILE"

    # 8. costmap 话题是否有数据发布
    echo "[COSTMAP] 局部代价地图话题:" | tee -a "$LOG_FILE"
    timeout 2 ros2 topic hz /local_costmap/costmap --window 3 2>&1 | \
        grep -E "average rate|no new messages" | head -1 | tee -a "$LOG_FILE"

    echo "[COSTMAP] 全局代价地图话题:" | tee -a "$LOG_FILE"
    timeout 2 ros2 topic hz /global_costmap/costmap --window 3 2>&1 | \
        grep -E "average rate|no new messages" | head -1 | tee -a "$LOG_FILE"

    sleep "$INTERVAL"
done
