#!/usr/bin/env python3
"""
简化的 GNSS 数据模拟器 - 可以直接写入文件或串口
"""

import time
import sys

# NMEA 数据（模拟 GPS 在成都市区的移动）
NMEA_SENTENCES = [
    "$GNGGA,063622.00,3046.05670,N,10359.01462,E,2,12,0.52,514.1,M,-30.0,M,,*63",
    "$GNGGA,063623.00,3046.05671,N,10359.01460,E,2,12,0.52,514.1,M,-30.0,M,,*61",
    "$GNGGA,063624.00,3046.05671,N,10359.01459,E,2,12,0.52,514.2,M,-30.0,M,,*6F",
    "$GNGGA,063625.00,3046.05671,N,10359.01457,E,2,12,0.52,514.2,M,-30.0,M,,*60",
    "$GNGGA,063626.00,3046.05672,N,10359.01456,E,2,12,0.51,514.3,M,-30.0,M,,*60",
    "$GNGGA,063627.00,3046.05673,N,10359.01455,E,2,12,0.51,514.3,M,-30.0,M,,*61",
]

def send_nmea_data(output, rate_hz=1.0):
    """
    发送 NMEA 数据

    Args:
        output: 输出对象（文件对象或串口对象）
        rate_hz: 发送频率
    """
    delay = 1.0 / rate_hz
    counter = 0

    try:
        while True:
            for sentence in NMEA_SENTENCES:
                data = sentence + "\r\n"

                if hasattr(output, 'write'):
                    # 文件或串口
                    output.write(data.encode('ascii') if hasattr(output, 'mode') and 'b' in output.mode else data)
                    if hasattr(output, 'flush'):
                        output.flush()
                else:
                    print(data, end='')

                counter += 1
                print(f"[{counter:04d}] 已发送: {sentence}", file=sys.stderr)
                time.sleep(delay)

    except KeyboardInterrupt:
        print(f"\n✓ 已停止（共发送 {counter} 条消息）", file=sys.stderr)

def main():
    import argparse

    parser = argparse.ArgumentParser(description='GNSS 数据模拟器')
    parser.add_argument('port', nargs='?', help='串口设备（如 /tmp/vgps0 或 /dev/ttyACM0）')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率（默认: 115200）')
    parser.add_argument('--rate', type=float, default=1.0, help='发送频率 Hz（默认: 1.0）')
    parser.add_argument('--stdout', action='store_true', help='输出到标准输出')

    args = parser.parse_args()

    if args.stdout:
        print("→ 输出到标准输出", file=sys.stderr)
        send_nmea_data(sys.stdout, args.rate)
    elif args.port:
        try:
            import serial
            ser = serial.Serial(args.port, args.baudrate, timeout=1)
            print(f"✓ 已打开串口: {args.port} @ {args.baudrate} baud", file=sys.stderr)
            send_nmea_data(ser, args.rate)
        except ImportError:
            print("❌ pyserial 未安装，请运行: pip3 install pyserial", file=sys.stderr)
            sys.exit(1)
        except Exception as e:
            print(f"❌ 错误: {e}", file=sys.stderr)
            sys.exit(1)
    else:
        parser.print_help()
        print("\n示例用法:", file=sys.stderr)
        print("  python3 simulate_gnss_simple.py /tmp/vgps0", file=sys.stderr)
        print("  python3 simulate_gnss_simple.py --stdout | cat", file=sys.stderr)

if __name__ == '__main__':
    main()
