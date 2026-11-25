#!/usr/bin/env python3
"""
GNSS 数据模拟器
模拟从串口发送 NMEA GPS 数据
"""

import serial
import time
import sys

# 示例 NMEA 数据（来自真实 GPS 接收器）
NMEA_DATA = [
    "$GNGLL,3046.05667,N,10359.01463,E,063621.00,A,D*7F",
    "$GNRMC,063622.00,A,3046.05670,N,10359.01462,E,0.024,,271125,,,D,V*1C",
    "$GNVTG,,T,,M,0.024,N,0.044,K,D*3E",
    "$GNGGA,063622.00,3046.05670,N,10359.01462,E,2,12,0.52,514.1,M,-30.0,M,,*63",
    "$GNGSA,A,3,09,31,26,27,08,16,04,03,,,,,0.97,0.52,0.82,1*00",
    "$GNGSA,A,3,21,29,27,30,19,,,,,,,,0.97,0.52,0.82,3*06",
    "$GNGSA,A,3,06,25,16,23,09,07,33,10,41,39,34,24,0.97,0.52,0.82,4*01",
    "$GNGSA,A,3,07,02,,,,,,,,,,,0.97,0.52,0.82,5*03",
    "$GPGSV,3,1,11,03,17,227,34,04,71,288,39,08,36,184,29,09,34,314,40,1*6E",
    "$GPGSV,3,2,11,16,65,014,41,26,33,047,40,27,57,145,41,31,33,085,42,1*69",
    "$GPGSV,3,3,11,41,47,217,38,42,51,205,36,50,46,140,35,1*59",

    "$GNGLL,3046.05670,N,10359.01462,E,063622.00,A,D*7B",
    "$GNRMC,063623.00,A,3046.05671,N,10359.01460,E,0.012,,271125,,,D,V*1B",
    "$GNVTG,,T,,M,0.012,N,0.022,K,D*3B",
    "$GNGGA,063623.00,3046.05671,N,10359.01460,E,2,12,0.52,514.1,M,-30.0,M,,*61",
    "$GNGSA,A,3,09,31,26,27,08,16,04,03,,,,,0.97,0.52,0.82,1*00",

    "$GNGLL,3046.05671,N,10359.01459,E,063624.00,A,D*74",
    "$GNRMC,063624.00,A,3046.05671,N,10359.01459,E,0.012,,271125,,,D,V*16",
    "$GNVTG,,T,,M,0.012,N,0.021,K,D*38",
    "$GNGGA,063624.00,3046.05671,N,10359.01459,E,2,12,0.52,514.2,M,-30.0,M,,*6F",

    "$GNGLL,3046.05671,N,10359.01457,E,063625.00,A,D*74",
    "$GNRMC,063625.00,A,3046.05671,N,10359.01457,E,0.010,,271125,,,D,V*1B",
    "$GNVTG,,T,,M,0.010,N,0.018,K,D*30",
    "$GNGGA,063625.00,3046.05671,N,10359.01457,E,2,12,0.52,514.2,M,-30.0,M,,*60",
]

def simulate_gps(port='/dev/ttyVGPS0', baudrate=115200, rate_hz=1.0):
    """
    模拟 GPS 数据发送

    Args:
        port: 串口设备路径
        baudrate: 波特率
        rate_hz: 发送频率（Hz）
    """
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ 已打开串口: {port} @ {baudrate} baud")
        print(f"✓ 发送频率: {rate_hz} Hz")
        print(f"✓ 开始发送 NMEA 数据...\n")

        delay = 1.0 / rate_hz
        counter = 0

        while True:
            # 循环发送 NMEA 数据
            for line in NMEA_DATA:
                # 添加回车换行
                data = (line + "\r\n").encode('ascii')
                ser.write(data)

                # 每发送一个 GNGGA 消息就打印信息
                if line.startswith("$GNGGA"):
                    counter += 1
                    print(f"[{counter:04d}] 已发送: {line}")

                time.sleep(delay / len(NMEA_DATA))

    except serial.SerialException as e:
        print(f"✗ 串口错误: {e}")
        print(f"\n提示：请先创建虚拟串口:")
        print(f"  sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyVGPS0 pty,raw,echo=0,link=/tmp/ttyVGPS1")
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n\n✓ 已停止发送（共发送 {counter} 条消息）")
        ser.close()
        sys.exit(0)

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/tmp/ttyVGPS0'

    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
    else:
        baudrate = 115200

    if len(sys.argv) > 3:
        rate_hz = float(sys.argv[3])
    else:
        rate_hz = 1.0  # 1 Hz (每秒发送一组数据)

    print("=" * 60)
    print("    GNSS 数据模拟器")
    print("=" * 60)

    simulate_gps(port, baudrate, rate_hz)

if __name__ == '__main__':
    main()
