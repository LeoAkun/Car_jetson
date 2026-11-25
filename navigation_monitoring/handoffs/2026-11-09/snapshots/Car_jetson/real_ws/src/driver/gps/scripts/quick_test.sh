#!/bin/bash
# 快速测试脚本 - 使用虚拟串口测试 GNSS Converter

echo "================================================"
echo "       GNSS Converter 快速测试"
echo "================================================"
echo ""

# 检查 socat
if ! command -v socat &> /dev/null; then
    echo "❌ socat 未安装"
    echo "请运行: sudo apt-get install socat"
    echo ""
    read -p "是否现在安装? (y/n): " install_socat
    if [ "$install_socat" = "y" ]; then
        sudo apt-get update && sudo apt-get install -y socat
    else
        exit 1
    fi
fi

# 检查 pyserial
if ! python3 -c "import serial" 2>/dev/null; then
    echo "❌ pyserial 未安装"
    echo "正在安装..."
    pip3 install pyserial --user
fi

echo "✅ 所有依赖已就绪"
echo ""

# 设置虚拟串口路径
VPORT0="/tmp/vgps0"
VPORT1="/tmp/vgps1"

# 清理旧的虚拟串口
pkill -f "socat.*vgps" 2>/dev/null || true
rm -f ${VPORT0} ${VPORT1}

# 创建虚拟串口对
echo "→ 创建虚拟串口对..."
socat -d -d pty,raw,echo=0,link=${VPORT0} pty,raw,echo=0,link=${VPORT1} &
SOCAT_PID=$!
sleep 2

if [ ! -e ${VPORT0} ]; then
    echo "❌ 虚拟串口创建失败"
    exit 1
fi

chmod 666 ${VPORT0} ${VPORT1} 2>/dev/null || true
echo "✅ 虚拟串口创建成功"
echo "   写入端: ${VPORT0}"
echo "   读取端: ${VPORT1}"
echo ""

# Source ROS2 环境
echo "→ 加载 ROS2 环境..."
cd /home/cplus/Desktop/yunle_sensors
source install/setup.bash
echo "✅ ROS2 环境已加载"
echo ""

# 启动 GNSS Converter 节点（后台运行）
echo "→ 启动 GNSS Converter 节点..."
ros2 run gnss_converter gnss_converter_node \
    --ros-args \
    -p serial_port:=${VPORT1} \
    -p baudrate:=115200 \
    > /tmp/gnss_converter.log 2>&1 &
CONVERTER_PID=$!

sleep 3
echo "✅ GNSS Converter 节点已启动 (PID: ${CONVERTER_PID})"
echo ""

# 启动数据模拟器（后台运行）
echo "→ 启动 GNSS 数据模拟器..."
python3 src/gnss_converter/scripts/simulate_gnss.py ${VPORT0} 115200 1.0 > /tmp/simulator.log 2>&1 &
SIMULATOR_PID=$!

sleep 3
echo "✅ GNSS 模拟器已启动 (PID: ${SIMULATOR_PID})"
echo ""

# 等待数据处理
echo "→ 等待数据处理 (5秒)..."
sleep 5

# 测试结果
echo ""
echo "================================================"
echo "       测试结果"
echo "================================================"
echo ""

# 检查话题
if ros2 topic list 2>/dev/null | grep -q "/sensing/gnss/navsat"; then
    echo "✅ 话题存在: /sensing/gnss/navsat"
    echo ""

    # 显示话题信息
    echo "📊 话题信息:"
    ros2 topic info /sensing/gnss/navsat
    echo ""

    # 显示频率
    echo "📈 话题频率:"
    timeout 3 ros2 topic hz /sensing/gnss/navsat 2>/dev/null || echo "  (等待数据...)"
    echo ""

    # 显示一条消息
    echo "📦 接收到的消息:"
    echo "----------------------------------------"
    timeout 5 ros2 topic echo /sensing/gnss/navsat --once 2>/dev/null || echo "  ❌ 无数据接收"
    echo "----------------------------------------"
    echo ""

    # 显示日志
    echo "📝 转换节点日志 (最后10行):"
    tail -10 /tmp/gnss_converter.log
    echo ""

    echo "✅✅✅ 测试通过！GNSS Converter 工作正常！"

else
    echo "❌ 话题不存在"
    echo ""
    echo "📝 转换节点日志:"
    cat /tmp/gnss_converter.log
    echo ""
    echo "📝 模拟器日志:"
    cat /tmp/simulator.log
fi

echo ""
echo "================================================"

# 询问是否继续运行
read -t 10 -p "保持运行以继续观察? (10秒后自动清理) [y/N]: " keep_running
echo ""

if [ "$keep_running" = "y" ] || [ "$keep_running" = "Y" ]; then
    echo "✅ 节点继续运行..."
    echo ""
    echo "💡 使用以下命令查看实时数据:"
    echo "   ros2 topic echo /sensing/gnss/navsat"
    echo ""
    echo "按 Ctrl+C 停止"

    trap "echo ''; echo '清理中...'; kill ${SIMULATOR_PID} ${CONVERTER_PID} ${SOCAT_PID} 2>/dev/null; rm -f ${VPORT0} ${VPORT1}; echo '✅ 已清理'; exit 0" INT

    wait
else
    echo "→ 清理进程..."
    kill ${SIMULATOR_PID} 2>/dev/null || true
    kill ${CONVERTER_PID} 2>/dev/null || true
    kill ${SOCAT_PID} 2>/dev/null || true
    rm -f ${VPORT0} ${VPORT1}
    echo "✅ 清理完成"
fi

echo ""
echo "================================================"
echo "       测试结束"
echo "================================================"
