"""
飞轮控制核心模块
"""
import serial
import time
import struct
from typing import Dict

class FlyWheel:
    """
    飞轮控制类，负责与飞轮硬件通过RS232进行通信
    """
    def __init__(self, port: str = 'COM7', baudrate: int = 115200):
        """
        初始化飞轮连接
        
        Args:
            port: RS232端口名称
            baudrate: 波特率
        """
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        self._is_connected = False
        
    def connect(self) -> bool:
        """
        建立与飞轮的连接
        """
        try:
            if not self.serial.is_open:
                self.serial.open()
            self._is_connected = True
            return True
        except Exception as e:
            print(f"连接失败: {str(e)}")
            self._is_connected = False
            return False

    def poll_status(self) -> Dict:
        """
        发送轮询包并读取状态
        """
        if not self._is_connected:
            raise ConnectionError("飞轮未连接")

        # 发送轮询命令
        command = bytes([0xEB, 0x90, 0xDD, 0x00, 0x00, 0x00, 0x00, 0xDD])
        self.serial.write(command)
        time.sleep(0.1)  # 等待响应
        
        # 读取32字节响应
        response = self.serial.read(32)
        return self._process_data(response)

    def set_speed(self, speed: float) -> bool:
        """
        设置飞轮转速
        
        Args:
            speed: 目标转速 (-6050 到 +6050 RPM)
        """
        if not -6050 <= speed <= 6050:
            raise ValueError("速度必须在 -6050 到 +6050 RPM 之间")

        command = self._build_speed_command(speed)
        self.serial.write(command)
        time.sleep(0.1)
        
        # 读取8字节响应
        response = self.serial.read(8)
        return len(response) == 8

    def _build_speed_command(self, speed: float) -> bytes:
        """
        构建速度命令包
        
        Args:
            speed: 目标转速
        """
        header = bytes([0xEB, 0x90])
        command_code = 0xD2
        
        # 将速度转换为IEEE 754格式
        speed_bytes = struct.pack('>f', float(speed))
        # 高位在前，不用反转
        speed_data = speed_bytes
        
        # 计算校验和
        checksum = (command_code + sum(speed_data)) & 0xFF
        
        return header + bytes([command_code]) + speed_data + bytes([checksum])

    def _process_data(self, data: bytes) -> Dict:
        """
        处理32字节的遥测数据
        
        Args:
            data: 32字节的原始数据
        """
        if len(data) != 32:
            raise ValueError("数据长度必须为32字节")

        telemetry = {
            'header': '0x' + ' '.join([f'{x:02X}' for x in data[0:3]]),
            'last_command': f'0x{data[3]:02X}',
            'control_target': struct.unpack('>f', bytes(reversed(data[4:8])))[0],
            'flywheel_speed_feedback': struct.unpack('>f', bytes(reversed(data[8:12])))[0],
            'flywheel_current_feedback': struct.unpack('>f', bytes(reversed(data[12:16])))[0],
            'acceleration_feedback': struct.unpack('>f', bytes(reversed(data[16:20])))[0],
            'command_response_count': data[20],
            'telemetry_wheel_command_count': data[21],
            'error_command_count': data[22],
            'motherboard_current': int.from_bytes(data[23:25], byteorder='big'),
            'temperature': int.from_bytes(bytes([data[25]]), byteorder='big', signed=True),
            'single_motor_status': data[26],
            'reserved': data[27:31].hex(),
            'checksum': data[31]
        }
        
        return telemetry

    def disconnect(self):
        """
        断开与飞轮的连接
        """
        if self.serial.is_open:
            self.serial.close()
        self._is_connected = False
