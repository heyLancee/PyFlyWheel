from pyflywheel.core import FlyWheel
import time
import logging


if __name__ == "__main__":
    COM = 'COM5'
    BAUD = 115200
    fw = FlyWheel(port=COM, baudrate=BAUD, auto_polling=True, polling_frequency=1000, communication_frequency=1000)
    logging.basicConfig(level=logging.DEBUG)
    fw.connect()
    fw.start()

    time.sleep(100)
    fw.stop()
    fw.disconnect()