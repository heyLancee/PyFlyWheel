import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import logging
import queue
import socket
import threading
from pyflywheel.core import FlyWheel
import scipy.io as sio
import numpy as np

# 配置日志记录
logging.basicConfig(level=logging.DEBUG)

def udp_sender(udp_queue, udp_ip, udp_port):
    """UDP发送线程函数"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        data = udp_queue.get()
        try:
            sock.sendto(data.encode(), (udp_ip, udp_port))
            logging.debug(f"Sent data: {data} to {udp_ip}:{udp_port}")
        except Exception as e:
            logging.error(f"UDP send failed: {e}")

def load_speed_profile(file_path):
    """直接加载速度配置文件"""
    try:
        mat_data = sio.loadmat(file_path)
        speeds = mat_data.get('speed_profile', [])
        
        # 数据预处理
        if isinstance(speeds, np.ndarray):
            speeds = speeds.flatten().tolist()
        else:
            raise ValueError("Invalid data format")
        
        # 过滤无效值
        valid_speeds = []
        for spd in speeds:
            if np.isreal(spd) and not np.isnan(spd):
                valid_speeds.append(int(spd))
        
        if not valid_speeds:
            raise ValueError("No valid speed data")
        
        # 创建预定义长度的队列
        speed_queue = queue.Queue(maxsize=len(valid_speeds))
        [speed_queue.put(spd) for spd in valid_speeds]
        
        return speed_queue, valid_speeds[-1]
    
    except Exception as e:
        logging.error(f"Failed to load speed profile: {str(e)}")
        sys.exit(1)

def main():
    # 配置参数
    COM = 'COM5'
    BAUD = 115200
    POLLING_FREQ = 100  # 主线程循环频率（Hz）
    UDP_IP = '127.0.0.1'  # 目标IP地址
    UDP_PORT = 5005       # 目标端口
    SPEED_FILE = 'speed_profile.mat'  # 转速文件路径

    # 预加载速度数据（主线程执行）
    speed_queue, last_speed = load_speed_profile(SPEED_FILE)
    logging.info(f"Loaded {speed_queue.qsize()} speed points, last speed: {last_speed} RPM")

    # 初始化FlyWheel实例
    fw = FlyWheel(port=COM, baudrate=BAUD, auto_polling=True,
                  polling_frequency=POLLING_FREQ, communication_frequency=1000)
    
    # 遥测数据队列
    telemetry_queue = queue.Queue(maxsize=10000)
    fw.callback = lambda x: telemetry_queue.put(x) if not telemetry_queue.full() else None

    # 启动UDP发送线程
    udp_thread = threading.Thread(target=udp_sender, args=(telemetry_queue, UDP_IP, UDP_PORT))
    udp_thread.daemon = True
    udp_thread.start()


    # 主控制变量
    current_speed = 0
    use_last_speed = False

    try:
        # 连接设备并启动
        fw.connect()
        fw.start()

        # 主循环（以polling_frequency频率运行）
        while True:
            if not use_last_speed:
                try:
                    # 尝试获取新速度指令（非阻塞）
                    current_speed = speed_queue.get_nowait()
                except queue.Empty:
                    # 队列为空时切换到最后一个值
                    use_last_speed = True
                    current_speed = last_speed
                    logging.info(f"Switching to last speed: {current_speed} RPM")

            # 设置飞轮速度
            fw.set_speed(current_speed)
            
            # 控制循环频率
            time.sleep(1.0 / POLLING_FREQ)

    except KeyboardInterrupt:
        logging.info("Stopping program...")
    except Exception as e:
        logging.error(f"Main loop error: {e}")
    finally:
        # 优雅退出
        fw.stop()
        fw.disconnect()
        logging.info("Resources released")

if __name__ == "__main__":
    main()