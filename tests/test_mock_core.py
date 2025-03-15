"""
飞轮控制模块测试套件
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import unittest
from unittest.mock import Mock, patch
import struct
from pyflywheel.core import FlyWheel
import queue


class TestFlyWheel(unittest.TestCase):
    """
    飞轮控制类测试
    """
    def setUp(self):
        """
        测试前准备
        """
        # 模拟串口
        self.mock_serial_patcher = patch('pyflywheel.core.serial.Serial')
        self.mock_serial = self.mock_serial_patcher.start()
        
        # 创建飞轮实例
        self.flywheel = FlyWheel(port='COM7', baudrate=115200)

        # 连接
        self.flywheel.serial.is_open = False
        self.flywheel.connect()
        self.flywheel.serial.open.assert_called_once()
        
    def tearDown(self):
        """
        测试后清理
        """
        # 确保通信线程停止
        self.flywheel.disconnect()
        
        # 停止mock
        self.mock_serial_patcher.stop()
        
    def test_init(self):
        """
        测试初始化参数
        """
        self.mock_serial.assert_called_with(
            port='COM7',
            baudrate=115200,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=1
        )
        
    def test_connect(self):
        """
        测试连接功能
        """
        # 设置mock
        self.flywheel.serial.is_open = False
        self.flywheel.serial.open = Mock(return_value=True)
        
        # 测试连接
        result = self.flywheel.connect()
        self.assertTrue(result)
        self.assertTrue(self.flywheel._is_connected)
        self.flywheel.serial.open.assert_called_once()
        
    def test_build_speed_command(self):
        """
        测试速度命令构建
        """
        # 测试速度100 RPM
        command = self.flywheel._build_speed_command(100.0)
        
        # 验证命令格式
        self.assertEqual(command[0:2], bytes([0xEB, 0x90]))  # 头部
        self.assertEqual(command[2], 0xD2)  # 命令码
        
        # 验证速度值（IEEE 754格式）
        speed_bytes = command[3:7]
        speed = struct.unpack('>f', bytes(speed_bytes))[0]
        self.assertAlmostEqual(speed, 100.0)
        
        # 验证校验和
        checksum = command[7]
        expected_checksum = (0xD2 + sum(speed_bytes)) & 0xFF
        self.assertEqual(checksum, expected_checksum)
        
    def test_set_speed_success(self):
        """
        测试正常设置速度
        """
        # 模拟正确的8字节响应
        self.flywheel.serial.read.return_value = b'\x00' * 8
        
        # 测试设置速度
        result = self.flywheel.set_speed(100.0)
        self.assertTrue(result)
        
        # 验证写入命令
        self.flywheel.serial.write.assert_called_once()
        
    def test_set_speed_failure(self):
        """
        测试设置速度失败情况
        """
        # 模拟错误响应（少于8字节）
        self.flywheel.serial.read.return_value = b'\x00' * 7
        
        # 测试设置速度
        result = self.flywheel.set_speed(100.0)
        self.assertFalse(result)
        
    def test_speed_limits(self):
        """
        测试速度限制
        """
        # 测试超出范围的速度
        with self.assertRaises(ValueError) as context:
            self.flywheel.set_speed(7000)
        self.assertEqual(str(context.exception), "速度必须在 -6050 到 +6050 RPM 之间")
        
        with self.assertRaises(ValueError) as context:
            self.flywheel.set_speed(-7000)
        self.assertEqual(str(context.exception), "速度必须在 -6050 到 +6050 RPM 之间")
            
        # 测试边界值
        try:
            # 测试最大速度
            self.flywheel.set_speed(6050)
            # 测试最小速度
            self.flywheel.set_speed(-6050)
            # 测试零速度
            self.flywheel.set_speed(0)
        except ValueError:
            self.fail("合法速度值引发了异常")
            
    def test_poll_status(self):
        """
        测试状态轮询
        """
        # 模拟32字节响应数据
        mock_response = (
            b'\xEB\x90\xDD'  # 头部
            b'\x00'  # last_command
            b'\x00\x00\x00\x00'  # control_target
            b'\x00\x00\x00\x00'  # flywheel_speed_feedback
            b'\x00\x00\x00\x00'  # flywheel_current_feedback
            b'\x00\x00\x00\x00'  # acceleration_feedback
            b'\x00'  # command_response_count
            b'\x00'  # telemetry_wheel_command_count
            b'\x00'  # error_command_count
            b'\x00\x00'  # motherboard_current
            b'\x00'  # temperature
            b'\x00'  # single_motor_status
            b'\x00\x00\x00\x00'  # reserved
            b'\x00'  # checksum
        )
        
        self.flywheel.serial.read.return_value = mock_response
        
        # 测试轮询
        status = self.flywheel.poll_status()
        
        # 验证发送的轮询命令
        self.flywheel.serial.write.assert_called_with(
            bytes([0xEB, 0x90, 0xDD, 0x00, 0x00, 0x00, 0x00, 0xDD])
        )
        
        # 验证解析的状态数据
        self.assertEqual(status['header'], '0xEB 90 DD')
        self.assertEqual(status['last_command'], '0x00')
        self.assertEqual(status['control_target'], 0.0)
        self.assertEqual(status['flywheel_speed_feedback'], 0.0)
        
    def test_process_data(self):
        """
        测试遥测数据处理
        """
        # 创建测试数据
        test_data = bytearray(32)
        test_data[0:3] = [0xEB, 0x90, 0xDD]  # 头部
        
        # 设置一个测试速度值 (100.0)
        speed_bytes = struct.pack('>f', 100.0)
        test_data[8:12] = speed_bytes  # flywheel_speed_feedback
        
        # 测试数据处理
        result = self.flywheel._process_data(test_data)
        
        # 验证结果
        self.assertEqual(result['header'], '0xEB 90 DD')
        self.assertAlmostEqual(result['flywheel_speed_feedback'], 100.0)
        
    def test_disconnect(self):
        """
        测试断开连接
        """
        self.flywheel.serial.is_open = True
        self.flywheel.disconnect()
        self.flywheel.serial.close.assert_called_once()
        self.assertFalse(self.flywheel._is_connected)
        
    def test_build_torque_command(self):
        """
        测试力矩命令构建
        """
        # 测试力矩 +30 mNm
        command = self.flywheel._build_torque_command(30.0)
        
        # 验证命令格式
        self.assertEqual(command[0:2], bytes([0xEB, 0x90]))  # 头部
        self.assertEqual(command[2], 0xD3)  # 命令码
        
        # 验证力矩值（IEEE 754格式）
        torque_bytes = command[3:7]
        torque = struct.unpack('>f', bytes(torque_bytes))[0]
        self.assertAlmostEqual(torque, 30.0)
        
        # 验证校验和
        checksum = command[7]
        expected_checksum = (0xD3 + sum(torque_bytes)) & 0xFF
        self.assertEqual(checksum, expected_checksum)
        
        # 验证完整命令
        self.assertEqual(command, bytes.fromhex('EB90D341F0000004'))
        
        # 测试力矩 -30 mNm
        command = self.flywheel._build_torque_command(-30.0)
        self.assertEqual(command, bytes.fromhex('EB90D3C1F0000084'))
        
    def test_set_torque_success(self):
        """
        测试正常设置力矩
        """
        # 模拟正确的8字节响应
        self.flywheel.serial.read.return_value = b'\x00' * 8
        
        # 测试设置力矩
        result = self.flywheel.set_torque(30.0)
        self.assertTrue(result)
        
        # 验证写入命令
        self.flywheel.serial.write.assert_called_once()
        
    def test_set_torque_failure(self):
        """
        测试设置力矩失败情况
        """
        # 模拟队列已满
        self.flywheel.queue.put = Mock(side_effect=queue.Full)
        
        # 测试设置力矩
        result = self.flywheel.set_torque(30.0)
        self.assertFalse(result)
        
    def test_torque_limits(self):
        """
        测试力矩限制
        """
        # 测试超出范围的力矩
        with self.assertRaises(ValueError) as context:
            self.flywheel.set_torque(60)
        self.assertEqual(str(context.exception), "力矩必须在 -50 到 +50 mNm 之间")
        
        with self.assertRaises(ValueError) as context:
            self.flywheel.set_torque(-60)
        self.assertEqual(str(context.exception), "力矩必须在 -50 到 +50 mNm 之间")
            
        # 测试边界值
        try:
            # 测试最大力矩
            self.flywheel.set_torque(50)
            # 测试最小力矩
            self.flywheel.set_torque(-50)
            # 测试零力矩
            self.flywheel.set_torque(0)
        except ValueError:
            self.fail("合法力矩值引发了异常")


if __name__ == '__main__':
    unittest.main()
