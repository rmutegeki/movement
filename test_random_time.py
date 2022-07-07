import numpy as np
import maestro
#from threading import Thread
import serial
import random
import subprocess
import time
import sys
sys.path.append('./')
from data_config import DataConfig

class MaicatControl:

    def __init__(self):
        self.next_time_to_action = self._time_to_perform_action()
	for sn in range(10):
	   self.next_time_to_action = self._time_to_perform_action()
	   print time.time(), self.next_time_to_action
	   time.sleep(0.5)
	exit()

    def _time_to_perform_action(self):
        delay_minutes = random.randint(120, 300) # 1-3 minutes 3m = 180s
        return time.time() + delay_minutes

if __name__ == '__main__':
    try:
        mc = MaicatControl()
        mc.start()
    except KeyboardInterrupt:
        print("Shutting down - MaicatControl")
        mc.stop()
