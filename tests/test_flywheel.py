import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
print("sys.path:", sys.path)
from pyflywheel.core import FlyWheel
import time
import logging

if __name__ == "__main__":
    COM = 'COM5'
    BAUD = 115200
    fw = FlyWheel(port=COM, baudrate=BAUD, auto_polling=True, polling_frequency=1e3, thread_frequency=1e3)
    logging.basicConfig(level=logging.DEBUG)
    fw.callback = lambda x: print(x)
    fw.connect()
    fw.start()


    fw.set_speed(200); #200转
    time.sleep(100)


    fw.stop()
    fw.disconnect()
