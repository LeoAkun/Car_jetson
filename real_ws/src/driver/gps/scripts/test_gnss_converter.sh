#!/bin/bash

# GNSS Converter 测试脚本
# 创建虚拟串口、启动模拟器和转换节点，验证数据转换是否正确

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 虚拟串口路径
VGPS_WRITER="/tmp/ttyVGPS0"
VGPS_READER="/tmp/ttyVGPS1"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}    GNSS Converter 测试脚本${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# 检查 socat 是否安装
if ! command -v socat &> /dev/null; then
    echo -e "${YELLOW}⚠ socat 未安装，正在安装...${NC}"
    sudo apt-get update
    sudo apt-get install -y socat
fi

# 检查 pyserial 是否安装
if ! python3 -c "import serial" 2>/dev/null; then
    echo -e "${YELLOW}⚠ pyserial 未安装，正在安装...${NC}"
    pip3 install pyserial
fi

# 清理可能存在的虚拟串口
echo -e "${YELLOW}→ 清理旧的虚拟串口...${NC}"
rm -f ${VGPS_WRITER} ${VGPS_READER}
pkill -f "socat.*ttyVGPS" || true

# 创建虚拟串口对
echo -e "${GREEN}✓ 创建虚拟串口对...${NC}"
echo -e "  写入端: ${VGPS_WRITER}"
echo -e "  读取端: ${VGPS_READER}"

socat -d -d pty,raw,echo=0,link=${VGPS_WRITER} pty,raw,echo=0,link=${VGPS_READER} &
SOCAT_PID=$!

# 等待串口创建完成
sleep 2

# 检查串口是否创建成功
if [ ! -e ${VGPS_WRITER} ] || [ ! -e ${VGPS_READER} ]; then
    echo -e "${RED}✗ 虚拟串口创建失败${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 虚拟串口创建成功${NC}"
echo ""

# 设置串口权限
chmod 666 ${VGPS_WRITER} ${VGPS_READER}

# Source ROS2 环境
echo -e "${YELLOW}→ 加载 ROS2 环境...${NC}"
source /home/cplus/Desktop/yunle_sensors/install/setup.bash
echo -e "${GREEN}✓ ROS2 环境已加载${NC}"
echo ""

# 启动 gnss_converter 节点（使用读取端）
echo -e "${YELLOW}→ 启动 GNSS Converter 节点...${NC}"
ros2 run gnss_converter gnss_converter_node --ros-args -p serial_port:=${VGPS_READER} -p baudrate:=115200 &
CONVERTER_PID=$!

# 等待节点启动
sleep 3

echo -e "${GREEN}✓ GNSS Converter 节点已启动 (PID: ${CONVERTER_PID})${NC}"
echo ""

# 启动模拟器（使用写入端）
echo -e "${YELLOW}→ 启动 GNSS 数据模拟器...${NC}"
python3 /home/cplus/Desktop/yunle_sensors/src/gnss_converter/scripts/simulate_gnss.py ${VGPS_WRITER} 115200 1.0 &
SIMULATOR_PID=$!

sleep 2
echo -e "${GREEN}✓ GNSS 数据模拟器已启动 (PID: ${SIMULATOR_PID})${NC}"
echo ""

# 等待几秒让数据流动
echo -e "${BLUE}→ 等待数据处理...${NC}"
sleep 5

# 检查话题是否有数据
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}    检查 ROS2 话题数据${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# 检查话题是否存在
if ros2 topic list | grep -q "/sensing/gnss/pose_with_covariance"; then
    echo -e "${GREEN}✓ 话题 /sensing/gnss/pose_with_covariance 存在${NC}"
    echo ""

    # 显示话题信息
    echo -e "${YELLOW}→ 话题信息:${NC}"
    ros2 topic info /sensing/gnss/pose_with_covariance
    echo ""

    # 显示话题频率
    echo -e "${YELLOW}→ 话题频率:${NC}"
    timeout 5 ros2 topic hz /sensing/gnss/pose_with_covariance || true
    echo ""

    # 显示一条消息
    echo -e "${YELLOW}→ 接收到的消息示例:${NC}"
    echo -e "${BLUE}--------------------------------------------${NC}"
    timeout 5 ros2 topic echo /sensing/gnss/pose_with_covariance --once
    echo -e "${BLUE}--------------------------------------------${NC}"
    echo ""

    echo -e "${GREEN}✓✓✓ 测试成功！GNSS Converter 正常工作！${NC}"
else
    echo -e "${RED}✗ 话题 /sensing/gnss/pose_with_covariance 不存在${NC}"
    echo -e "${RED}✗ 测试失败${NC}"
fi

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}    清理测试环境${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# 询问是否保持运行
echo -e "${YELLOW}是否保持节点运行以继续观察？(y/n)${NC}"
read -t 10 -n 1 KEEP_RUNNING || KEEP_RUNNING="n"
echo ""

if [ "$KEEP_RUNNING" != "y" ]; then
    # 清理进程
    echo -e "${YELLOW}→ 停止所有进程...${NC}"
    kill ${SIMULATOR_PID} 2>/dev/null || true
    kill ${CONVERTER_PID} 2>/dev/null || true
    kill ${SOCAT_PID} 2>/dev/null || true

    # 清理虚拟串口
    rm -f ${VGPS_WRITER} ${VGPS_READER}

    echo -e "${GREEN}✓ 清理完成${NC}"
else
    echo -e "${GREEN}✓ 节点继续运行...${NC}"
    echo -e "${YELLOW}→ 使用以下命令查看实时数据:${NC}"
    echo -e "  ros2 topic echo /sensing/gnss/pose_with_covariance"
    echo ""
    echo -e "${YELLOW}→ 按 Ctrl+C 停止所有进程${NC}"

    # 等待用户中断
    wait
fi

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}    测试完成${NC}"
echo -e "${GREEN}============================================${NC}"
