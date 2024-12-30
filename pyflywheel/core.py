"""
飞轮控制核心模块
"""
from collections import deque
import serial
import time
import struct
import threading
from typing import Dict, Callable
from queue import Queue, Empty, Full
import logging


class FlyWheel:
    """
    飞轮控制类，负责与飞轮硬件通过RS232进行通信
    """
    def __init__(self, port: str = 'COM7', baudrate: int = 115200, queue_size: int = 10, thread_frequency: int = 100, 
                 callback: Callable[[Dict], None] = None, max_telemetry_size: int = 1000,
                 logger: logging.Logger = None):
        """
        初始化飞轮连接
        
        Args:
            port: RS232端口名称
            baudrate: 波特率
            queue_size: 通信队列大小
            thread_frequency: 通信线程频率
            callback: 回调函数,接收遥测数据字典,可用于实时控制转速
            max_telemetry_size: 遥测数据最大存储数量
            logger: 自定义日志记录器
        """
        self.logger = logger or logging.getLogger(__name__)
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        self._is_connected = False

        # 通信队列
        self.queue = Queue(maxsize=queue_size)

        # 通信线程频率
        self.thread_frequency = thread_frequency

        # 启动一个线程，处理与飞轮的通信
        self._running = True
        self._comm_thread = threading.Thread(target=self._communication_loop)
        self._comm_thread.start()

        # 回调函数，当收到32字节的遥测数据时，调用回调函数
        self.callback = callback

        # 存储遥测数据
        self.telemetry = deque(maxlen=max_telemetry_size)

    def __del__(self):
        """
        析构函数，确保资源正确释放
        """
        self.disconnect()

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

    def poll_status(self) -> bool:
        """
        发送轮询包并读取状态
        """
        if not self._is_connected:
            raise ConnectionError("飞轮未连接")

        # 发送轮询命令
        command = bytes([0xEB, 0x90, 0xDD, 0x00, 0x00, 0x00, 0x00, 0xDD])
        try:
            self.queue.put(command, timeout=1.0)
            return True
        except Full:
            return False

    def set_speed(self, speed: float) -> bool:
        """
        设置飞轮转速
        
        Args:
            speed: 目标转速 (-6050 到 +6050 RPM)
        """
        if not -6050 <= speed <= 6050:
            raise ValueError("速度必须在 -6050 到 +6050 RPM 之间")

        command = self._build_speed_command(speed)
        try:
            self.queue.put(command, timeout=1.0)
            return True
        except Full:
            return False

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
            'control_target': struct.unpack('>f', bytes(data[4:8]))[0],
            'flywheel_speed_feedback': struct.unpack('>f', bytes(data[8:12]))[0],
            'flywheel_current_feedback': struct.unpack('>f', bytes(data[12:16]))[0],
            'acceleration_feedback': struct.unpack('>f', bytes(data[16:20]))[0],
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
        # 停止通信线程
        self._running = False
        if hasattr(self, '_comm_thread'):
            self._comm_thread.join()
        
        # 关闭串口
        if self.serial.is_open:
            self.serial.close()
        self._is_connected = False

    def _communication_loop(self) -> None:
        """
        通信循环
        """
        while self._running:
            try:
                if not self._is_connected:
                    self.logger.warning("飞轮未连接")
                    time.sleep(1)
                    continue

                try:
                    command = self.queue.get(timeout=1/self.thread_frequency)
                    self.serial.write(command)
                except Empty:
                    continue
                except serial.SerialException as e:
                    self.logger.error(f"串口通信错误: {e}")
                    self._is_connected = False
                    continue

                # 固定频率
                time.sleep(1/self.thread_frequency)
                
                # 处理响应
                response = self.serial.read_all()
                if len(response) == 8:
                    continue
                elif len(response) == 32:
                    try:
                        telemetry = self._process_data(response)
                        if self.callback:
                            self.callback(telemetry)
                        self.telemetry.append((time.time(), telemetry))
                    except Exception as e:
                        print(f"处理遥测数据错误: {str(e)}")
                else:
                    # 清空缓冲区
                    self.serial.reset_input_buffer()

            except Exception as e:
                self.logger.exception(f"通信循环错误: {e}")
                time.sleep(1/self.thread_frequency)