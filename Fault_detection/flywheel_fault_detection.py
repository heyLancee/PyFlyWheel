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
import faulthandler
faulthandler.enable()



# 配置日志记录
log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

# 创建 FileHandler 并指定 UTF-8 编码
file_handler = logging.FileHandler("telemetry.log", encoding="utf-8")
file_handler.setFormatter(log_formatter)

# 获取 Logger 并添加 Handler
logger = logging.getLogger()
logger.setLevel(logging.INFO)
logger.addHandler(file_handler)

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
        speeds = mat_data.get('omega_ref', [])
        
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


def print_telemetry(telemetry, last_telemetry):
    """
    将当前时刻和上一时刻的遥测数据写入日志文件，并记录这是第几个数据点。
    
    参数:
        telemetry (TelemetryData): 当前时刻的遥测数据。
        last_telemetry (TelemetryData): 上一时刻的遥测数据。
    """
    # 使用函数属性记录调用次数（即数据点编号）
    if not hasattr(print_telemetry, "data_point_count"):
        print_telemetry.data_point_count = 0  # 初始化数据点编号为 0
    print_telemetry.data_point_count += 1  # 每次调用时递增编号

    # 构造日志消息
    log_message = (
        f"数据点编号: {print_telemetry.data_point_count}\n"
        f"当前时刻遥测数据:\n"
        f"  control_target: {telemetry.control_target}\n"
        f"  flywheel_speed_feedback: {telemetry.flywheel_speed_feedback}\n"
        f"  flywheel_current_feedback: {telemetry.flywheel_current_feedback}\n"
    )
    # 写入日志
    # logging.info(log_message)
    print(log_message)

def main():

    # 配置参数
    COM = 'COM5'
    BAUD = 115200
    POLLING_FREQ = 200  # 主线程循环频率（Hz）
    UDP_IP = '127.0.0.1'  # 目标IP地址
    UDP_PORT = 5005       # 目标端口
    SPEED_FILE = 'command_ref.mat'  # 转速文件路径

    # 预加载速度数据（主线程执行）
    speed_queue, last_speed = load_speed_profile(SPEED_FILE)
    logging.info(f"Loaded {speed_queue.qsize()} speed points, last speed: {last_speed} RPM")

    # 初始化FlyWheel实例
    fw = FlyWheel(port=COM, baudrate=BAUD, auto_polling=True,
                  polling_frequency=POLLING_FREQ, communication_frequency=POLLING_FREQ, timeout=None, queue_size=2000)
    
    # 遥测数据队列
    telemetry_queue = queue.Queue(maxsize=10000)
    fw.callback = print_telemetry

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
            fw.set_speed(500)
            
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