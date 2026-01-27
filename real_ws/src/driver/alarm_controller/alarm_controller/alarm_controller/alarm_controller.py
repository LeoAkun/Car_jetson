import serial
import time
import rclpy
from rclpy.node import Node
from alarm_interface.srv import Alarm

# Modbus CRC16 计算函数
def modbus_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

# 构建写单个线圈命令 (功能码 0x05)
def build_command(value):
    # address: 设备地址, 默认1
    # coil_addr: 线圈地址 (0-based, 如输出通道0对应0x0020即32)
    # value: True for ON (0xFF00), False for OFF (0x0000)
    cmd_off = b"\x01\x10\x01\xf4\x00\x01\x02\x00\x03"
    cmd_on = b"\x01\x10\x01\xf4\x00\x01\x02\x00\x04"
    if value == True:
        cmd = cmd_on
    else:
        cmd = cmd_off
    crc = modbus_crc16(cmd)
    return cmd + crc

class ModbusServer(Node):
    def __init__(self):
        super().__init__('modbus_server')
        self.srv = self.create_service(
            Alarm,
            '/control_alarm',
            self.control_alarm_callback
        )
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

    def control_alarm_callback(self, request, response):
        command = build_command(request.operate)

        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1
            )
            ser.write(command)
            resp_bytes = ser.read(8)  # 预期响应8字节 (6 + CRC2)
            ser.close()
            self.get_logger().info(f'command:{command.hex()}')
            self.get_logger().info(f'resp_bytes:{resp_bytes.hex()}')
            if len(resp_bytes) == 8:
                response.success = True
                response.message = f"命令发送成功，响应: {resp_bytes.hex()}"
            else:
                response.success = False
                response.message = "无响应或响应无效"
        except Exception as e:
            response.success = False
            response.message = f"串口错误: {str(e)}"

        return response

def main(args=None):
    rclpy.init(args=args)
    modbus_server = ModbusServer()
    rclpy.spin(modbus_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# 011001f40001020003e3e5 关闭警报
# 011001f40001020004a227 打开警报