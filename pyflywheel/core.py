"""
飞轮控制核心模块
"""
from collections import deque
import serial
import time
import struct
import threading
from typing import Dict, Callable, Optional
from queue import Queue, Empty, Full
import logging
import json
from datetime import datetime


class FlyWheel:
    """
    飞轮控制类，负责与飞轮硬件通过RS232进行通信
    """
    def __init__(self, port: str, baudrate: int, timeout: int = 1, inertia: float = 0.0001608,  
                 queue_size: int = 10, communication_frequency: int = 100, callback: Callable[[Dict, Dict], None] = None, 
                 max_telemetry_size: int = 1000, 
                 auto_polling: bool = False, polling_frequency: float = 10.0):
        """
        初始化飞轮连接
        
        Args:
            port: RS232端口名称
            baudrate: 波特率
            timeout: 串口超时时间
            queue_size: 通信队列大小
            communication_frequency: 通信线程频率
            callback: 回调函数,接收遥测数据字典,可用于实时控制转速
            max_telemetry_size: 遥测数据最大存储数量
            auto_polling: 是否自动开启轮询线程
            polling_frequency: 轮询频率，单位Hz
        """
        self.inertia = inertia

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.logger = logging.getLogger(__name__)
        
        self.cmd_queue = Queue(maxsize=queue_size)
        self.resp_len_queue = Queue(maxsize=queue_size)

        self._communication_frequency = communication_frequency
        self._polling_frequency = polling_frequency

        self._comm_thread = None
        self._polling_thread = None
        self._resp_thread = None

        self.auto_polling = auto_polling

        self._polling = False
        self._running = False
        self._is_connected = False

        self.callback = callback
        self.telemetry = deque(maxlen=max_telemetry_size)

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )

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
        
    def start(self):
        """
        启动飞轮通信和轮询
        """
        self._running = True
        
        # 启动通信线程
        self._comm_thread = threading.Thread(target=self._communication_loop, daemon=True)
        self._comm_thread.start()
        self.logger.info(f"已启动通信线程, 频率: {self._communication_frequency}Hz")
        
        # 启动响应处理线程
        self._resp_thread = threading.Thread(target=self._response_loop, daemon=True)
        self._resp_thread.start()
        self.logger.info(f"已启动响应处理线程, 频率: {self._communication_frequency}Hz")

        # 如果设置了自动轮询，则启动轮询线程
        if self.auto_polling:
            if self._polling:
                self.logger.warning("轮询线程已在运行")
                return False
        
            self._polling = True
            self._polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
            self._polling_thread.start()
            self.logger.info(f"已启动轮询线程，频率: {self._polling_frequency}Hz")

    def stop(self):
        """
        停止飞轮通信和轮询
        """
        self._running = False
        self._polling = False
        
        # 等待所有线程结束
        if self._comm_thread is not None:
            self._comm_thread.join()
        if self._resp_thread is not None:
            self._resp_thread.join()
        if self._polling_thread is not None:
            self._polling_thread.join()

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

    def set_current(self, current: float) -> bool:
        """
        设置飞轮电流
        
        Args:
            current: 目标电流，单位 mA，范围 -1500 到 +1500 mA
            
        Returns:
            bool: 是否成功将命令放入队列
            
        Raises:
            ValueError: 当电流超出范围时
        """
        if not -1500 <= current <= 1500:
            raise ValueError("电流必须在 -1500 到 +1500 mA 之间")
        
        command = self._build_current_command(current)
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

    def _build_current_command(self, current: float) -> bytes:
        """
        构建电流命令包
        
        Args:
            current: 目标电流，单位 mA
            
        Returns:
            bytes: 命令字节序列
        """
        header = bytes([0xEB, 0x90])
        command_code = 0xD1
        
        # 将电流转换为IEEE 754格式
        current_bytes = struct.pack('>f', float(current))
        
        # 计算校验和
        checksum = (command_code + sum(current_bytes)) & 0xFF
        
        return header + bytes([command_code]) + current_bytes + bytes([checksum])

    def _process_data(self, data: bytes) -> Dict:
        """
        处理32字节的遥测数据
        
        Args:
            data: 32字节的原始数据
        """
        if len(data) != 32:
            raise ValueError("数据长度必须为32字节")

        telemetry = {
            'timestamp': time.time(),
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

        # 等待通信线程完成
        if self._comm_thread is not None and self._comm_thread.is_alive():
            self._comm_thread.join()
        if self._polling_thread and self._polling_thread.is_alive():
            self._polling_thread.join()
        
        if self.serial is not None and self.serial.is_open:
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
                for data in self.telemetry:
                    telemetry_list.append(data)
                
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
        通信循环，只负责发送命令
        """
        period = 1.0 / self._communication_frequency
        next_time = time.perf_counter() + period
        
        while self._running:
            try:
                if not self._is_connected:
                    time.sleep(1)
                    continue
                
                try:
                    command = self.cmd_queue.get(timeout=self.timeout)
                except Empty:
                    continue

                self.serial.write(command)
                
                # 根据命令类型设置预期响应长度
                if command[2] == int('0xDD', 16):
                    self.resp_len_queue.put(32)
                else:
                    self.resp_len_queue.put(8)
                    
                next_time = self._wait_for_next_cycle(next_time, period)

            except Exception as e:
                self.logger.exception(f"通信循环错误: {e}")

    def _response_loop(self) -> None:
        """
        响应处理循环，专门处理串口响应数据
        """
        period = 1.0 / self._communication_frequency  # 使用与通信相同的频率
        next_time = time.perf_counter() + period

        while self._running:
            try:
                if not self._is_connected:
                    time.sleep(1)
                    next_time = time.perf_counter() + period  # 重置时间
                    continue
                    
                # 获取预期响应长度
                try:
                    resp_len = self.resp_len_queue.get(timeout=self.timeout)
                except Empty:
                    continue
                
                # 读取指定长度的响应
                if self.serial.in_waiting < resp_len:
                    continue
                response = self.serial.read(resp_len)
                
                if len(response) != resp_len:
                    self.logger.error(f"响应长度不匹配: 预期{resp_len}字节, 实际接收{len(response)}字节")
                    self.serial.reset_input_buffer()
                    next_time = self._wait_for_next_cycle(next_time, period)
                    continue
                
                # 处理32字节的遥测数据
                if len(response) == 32:
                    try:
                        telemetry = self._process_data(response)
                        if self.callback:
                            last_telemetry = self.telemetry[-1] if self.telemetry else telemetry
                            self.callback(telemetry, last_telemetry)
                        self.telemetry.append(telemetry)
                    except Exception as e:
                        self.logger.error(f"处理遥测数据错误: {str(e)}")
                
                next_time = self._wait_for_next_cycle(next_time, period)

            except Exception as e:
                self.logger.exception(f"响应处理循环错误: {e}")
                next_time = self._wait_for_next_cycle(next_time, period)

    def _wait_for_next_cycle(self, next_time: float, period: float) -> float:
        """
        等待下一个周期
        """
        current_time = time.perf_counter()
        sleep_time = next_time - current_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        current_time = time.perf_counter()
        next_time = current_time + period
            
        return next_time
    
    def _polling_loop(self) -> None:
        """
        轮询循环，使用时间累加器确保固定频率
        """
        period = 1.0 / self._polling_frequency  # 计算周期
        next_time = time.perf_counter() + period  # 使用高精度计时器

        while self._polling and self._running:
            try:
                if not self._is_connected:
                    time.sleep(1)
                    next_time = time.perf_counter() + period  # 重置时间
                    continue
                
                # 发送轮询命令
                self.poll_status()
                
                # 精确等待到下一个周期
                next_time = self._wait_for_next_cycle(next_time, period)
                    
            except Exception as e:
                self.logger.error(f"轮询过程发生错误: {str(e)}")

                
    def _send_command(self, command: bytes) -> bool:
        """
        发送命令到队列
        
        Args:
            command: 要发送的命令字节序列
            
        Returns:
            bool: 是否成功发送
        """
        try:
            self.cmd_queue.put_nowait(command)
            return True
        except Full:
            self.logger.error("命令队列已满")
            return False
