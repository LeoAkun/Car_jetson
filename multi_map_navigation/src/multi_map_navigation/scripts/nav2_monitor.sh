#!/bin/bash
# nav2_monitor.sh
# 导航系统运行状态监控脚本
# 用于诊断导航中 costmap 膨胀层消失、走走停停、程序崩溃等问题
#
# 监控内容:
#   1. TF 变换延迟 (map->base_link)
#   2. 激光雷达话题频率
#   3. cmd_vel 话题频率（判断是否在发速度）
#   4. CPU / 内存占用
#   5. Nav2 相关节点存活状态
#
# 使用方式:
#   chmod +x nav2_monitor.sh
#   ./nav2_monitor.sh
#
# 输出会同时显示在终端和写入日志文件 ~/nav2_monitor.log

LOG_FILE="$HOME/nav2_monitor.log"
INTERVAL=3  # 采样间隔（秒）

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================" | tee -a "$LOG_FILE"
echo " Nav2 导航系统监控脚本" | tee -a "$LOG_FILE"
echo " 采样间隔: ${INTERVAL}s" | tee -a "$LOG_FILE"
echo " 日志文件: ${LOG_FILE}" | tee -a "$LOG_FILE"
echo " 按 Ctrl+C 停止" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

while true; do
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    echo "" | tee -a "$LOG_FILE"
    echo "--- [$TIMESTAMP] ---" | tee -a "$LOG_FILE"

    # 1. TF 延迟检测: map -> base_link
    echo "[TF 延迟]" | tee -a "$LOG_FILE"
    TF_OUTPUT=$(timeout 2 ros2 run tf2_ros tf2_echo map base_link --wait-for-transform 1.0 2>&1 | head -5)
    if echo "$TF_OUTPUT" | grep -q "Translation"; then
        echo -e "  ${GREEN}map->base_link: 正常${NC}" | tee -a "$LOG_FILE"
        echo "  $TF_OUTPUT" | grep "Translation\|Rotation" | head -2 | tee -a "$LOG_FILE"
    else
        echo -e "  ${RED}map->base_link: 异常/超时${NC}" | tee -a "$LOG_FILE"
        echo "  $TF_OUTPUT" | head -3 | tee -a "$LOG_FILE"
    fi

    # 2. 话题频率检测
    echo "[话题频率]" | tee -a "$LOG_FILE"

    # 激光雷达
    SCAN_HZ=$(timeout 2 ros2 topic hz /laser/scan_merge_laser --window 5 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$SCAN_HZ" ]; then
        echo -e "  /laser/scan_merge_laser: ${GREEN}${SCAN_HZ} Hz${NC}" | tee -a "$LOG_FILE"
    else
        echo -e "  /laser/scan_merge_laser: ${RED}无数据或频率过低${NC}" | tee -a "$LOG_FILE"
    fi

    # cmd_vel（判断 Nav2 是否在发速度指令）
    CMD_HZ=$(timeout 2 ros2 topic hz /cmd_vel --window 5 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$CMD_HZ" ]; then
        echo -e "  /cmd_vel: ${GREEN}${CMD_HZ} Hz${NC}" | tee -a "$LOG_FILE"
    else
        echo -e "  /cmd_vel: ${YELLOW}无数据（可能已停车或未发布）${NC}" | tee -a "$LOG_FILE"
    fi

    # 3. Nav2 关键节点存活检测
    echo "[节点存活]" | tee -a "$LOG_FILE"
    NAV2_NODES="controller_server planner_server bt_navigator"
    NODE_LIST=$(ros2 node list 2>/dev/null)
    for NODE_NAME in $NAV2_NODES; do
        if echo "$NODE_LIST" | grep -q "$NODE_NAME"; then
            echo -e "  $NODE_NAME: ${GREEN}运行中${NC}" | tee -a "$LOG_FILE"
        else
            echo -e "  $NODE_NAME: ${RED}未检测到${NC}" | tee -a "$LOG_FILE"
        fi
    done

    # 4. 系统资源
    echo "[系统资源]" | tee -a "$LOG_FILE"

    # CPU 占用 top 3 进程
    CPU_INFO=$(ps aux --sort=-%cpu | head -4 | tail -3 | awk '{printf "  %s %.1f%% CPU %.1f%% MEM\n", $11, $3, $4}')
    echo "$CPU_INFO" | tee -a "$LOG_FILE"

    # 总体 CPU 和内存
    CPU_TOTAL=$(top -bn1 | grep "Cpu(s)" | awk '{print 100 - $8}')
    MEM_USED=$(free -m | awk '/Mem:/{printf "%.0f/%dMB (%.1f%%)", $3, $2, $3/$2*100}')
    echo "  CPU总占用: ${CPU_TOTAL}%" | tee -a "$LOG_FILE"
    echo "  内存: ${MEM_USED}" | tee -a "$LOG_FILE"

    # 如果有 tegrastats 可用（Jetson 平台）
    if command -v tegrastats &> /dev/null; then
        TEGRA=$(timeout 1 tegrastats --interval 500 2>/dev/null | head -1)
        if [ -n "$TEGRA" ]; then
            echo "  [tegrastats] $TEGRA" | tee -a "$LOG_FILE"
        fi
    fi

    # 5. local costmap 话题是否在发布（判断膨胀层是否活跃）
    echo "[Costmap 状态]" | tee -a "$LOG_FILE"
    COSTMAP_HZ=$(timeout 2 ros2 topic hz /local_costmap/costmap --window 3 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$COSTMAP_HZ" ]; then
        echo -e "  /local_costmap/costmap: ${GREEN}${COSTMAP_HZ} Hz${NC}" | tee -a "$LOG_FILE"
    else
        echo -e "  /local_costmap/costmap: ${RED}无更新${NC}" | tee -a "$LOG_FILE"
    fi

    sleep "$INTERVAL"
done
