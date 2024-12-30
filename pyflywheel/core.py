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
import pickle
import json
from datetime import datetime


class FlyWheel:
    """
    飞轮控制类，负责与飞轮硬件通过RS232进行通信
    """
    def __init__(self, inertia: float = 0.001608, port: str = 'COM5', baudrate: int = 115200, 
                 queue_size: int = 10, thread_frequency: int = 100, callback: Callable[[Dict], None] = None, 
                 max_telemetry_size: int = 1000, logger: logging.Logger = None, 
                 auto_polling: bool = False, polling_frequency: float = 50.0):
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
            auto_polling: 是否自动开启轮询线程
            polling_frequency: 轮询频率，单位Hz
        """
        # 惯量
        self.inertia = inertia
        
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

        # 添加条件变量
        self._comm_condition = threading.Condition()
        self.queue = Queue(maxsize=queue_size)

        # 通信线程频率
        self.thread_frequency = thread_frequency

        # 启动一个线程，处理与飞轮的通信
        self._running = True
        self._comm_thread = threading.Thread(target=self._communication_loop, daemon=True)
        self._comm_thread.start()

        # 回调函数，当收到32字节的遥测数据时，调用回调函数
        self.callback = callback

        # 存储遥测数据
        self.telemetry = deque(maxlen=max_telemetry_size)

        # 轮询相关属性
        self._polling_frequency = polling_frequency
        self._polling_thread = None
        self._polling = False
        
        # 如果设置了自动轮询，则启动轮询线程
        if auto_polling:
            self.start_polling()

    def __del__(self):
        """
        析构函数，确保资源正确释放
        """
        self.disconnect()

    def connect(self) -> bool:
        """
        建立与飞轮的连接

        Returns:
            bool: 是否成功连接

        Raises:
            ConnectionError: 当连接失败时
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

        Returns:
            bool: 是否成功将命令放入队列

        Raises:
            ConnectionError: 当飞轮未连接时
        """
        if not self._is_connected:
            raise ConnectionError("飞轮未连接")
        
        command = bytes([0xEB, 0x90, 0xDD, 0x00, 0x00, 0x00, 0x00, 0xDD])
        return self._send_command(command)

    def set_speed(self, speed: float) -> bool:
        """
        设置飞轮转速
        
        Args:
            speed: 目标转速 (-6050 到 +6050 RPM)

        Returns:
            bool: 是否成功将命令放入队列
            
        Raises:
            ValueError: 当速度超出范围时
        """
        if not -6050 <= speed <= 6050:
            raise ValueError("速度必须在 -6050 到 +6050 RPM 之间")
        
        command = self._build_speed_command(speed)
        return self._send_command(command)
        
    def set_torque(self, torque: float) -> bool:
        """
        设置飞轮力矩
        
        Args:
            torque: 目标力矩，单位 mNm，范围 -50 到 +50 mNm
            
        Returns:
            bool: 是否成功将命令放入队列
            
        Raises:
            ValueError: 当力矩超出范围时
        """
        # 验证力矩范围
        if not -50 <= torque <= 50:
            raise ValueError("力矩必须在 -50 到 +50 mNm 之间")
        
        command = self._build_torque_command(torque)
        return self._send_command(command)

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

    def _build_torque_command(self, torque: float) -> bytes:
        """
        构建力矩命令包
        
        Args:
            torque: 目标力矩，单位 mNm
            
        Returns:
            bytes: 命令字节序列
        """
        header = bytes([0xEB, 0x90])
        command_code = 0xD3
        
        # 将力矩转换为IEEE 754格式
        torque_bytes = struct.pack('>f', float(torque))
        
        # 计算校验和
        checksum = (command_code + sum(torque_bytes)) & 0xFF
        
        return header + bytes([command_code]) + torque_bytes + bytes([checksum])

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
        self._running = False  # 这会终止所有线程
        self._polling = False  # 为了明确性,也设置轮询标志
        
        if self.serial.is_open:
            self.serial.close()
        self._is_connected = False

    def save_telemetry(self, filename: str, format: str = 'json') -> bool:
        """
        保存遥测数据到文件

        Args:
            filename: 保存文件的路径
            format: 保存格式，支持 'json' 或 'csv'

        Returns:
            bool: 是否保存成功
        """
        try:
            if format.lower() == 'json':
                # 转换成可序列化的格式
                telemetry_list = []
                for timestamp, data in self.telemetry:
                    telemetry_dict = {
                        'timestamp': datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S.%f'),
                        'data': data
                    }
                    telemetry_list.append(telemetry_dict)
                
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(telemetry_list, f, indent=4, ensure_ascii=False)
            else:
                raise ValueError("不支持的格式，请使用 'json' 或 'csv'")
            
            self.logger.info(f"遥测数据已保存到 {filename}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存遥测数据失败: {str(e)}")
            return False

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

                # 使用条件变量等待命令
                with self._comm_condition:
                    while self.queue.empty() and self._running:
                        self._comm_condition.wait(timeout=1.0)  # 设置超时防止永久阻塞
                    
                    if not self._running:  # 检查是否需要退出
                        break
                        
                    try:
                        command = self.queue.get_nowait()  # 不需要timeout了
                        self.serial.write(command)
                    except Empty:  # 以防万一
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

    def start_polling(self) -> bool:
        """
        启动轮询线程
        
        Returns:
            bool: 是否成功启动轮询
        """
        if self._polling:
            self.logger.warning("轮询线程已在运行")
            return False
        
        self._polling = True
        self._polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
        self._polling_thread.start()
        self.logger.info(f"已启动轮询线程，频率: {self._polling_frequency}Hz")
        return True

    def _polling_loop(self) -> None:
        """
        轮询循环
        """
        while self._polling and self._running:
            try:
                if not self._is_connected:
                    self.logger.warning("飞轮未连接，轮询暂停")
                    time.sleep(1)
                    continue
                
                self.poll_status()
                time.sleep(1 / self._polling_frequency)
                
            except Exception as e:
                self.logger.error(f"轮询过程发生错误: {str(e)}")
                time.sleep(1)
                
    def _send_command(self, command: bytes) -> bool:
        """
        发送命令到队列并通知通信线程
        
        Args:
            command: 要发送的命令字节序列
            
        Returns:
            bool: 是否成功发送
        """
        try:
            self.queue.put_nowait(command)
            with self._comm_condition:
                self._comm_condition.notify()
            return True
        except Full:
            self.logger.error("命令队列已满")
            return False
                