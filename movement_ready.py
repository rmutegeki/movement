#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact
'''

import maestro
import time
import numpy as np
import subprocess
import serial
import argparse
import sys, os
import subprocess
sys.path.append('./')
from data_config import DataConfig
from optset_config import OptsetConfig


parser = argparse.ArgumentParser(description='')
parser.add_argument('-c', required=True, help='servo active 1, close 0')
args = parser.parse_args()

class MoveACT:

    def __init__(self):
        self.pwm = maestro.Controller()
        time.sleep(1)

        self.teensy_io = serial.Serial("/dev/ttyACM0", 9600)
        self.teensy_io.flush()

	self.joint_gap = OptsetConfig.JOINT_GAP_0

	joint_m = np.zeros((6, 64))
	joint_m[0] = DataConfig.JOINT_ready
	joint_m[1] = DataConfig.JOINT_0020
	joint_m[2] = DataConfig.JOINT_0021
	joint_m[3] = DataConfig.JOINT_0022
	joint_m[4] = DataConfig.JOINT_0028
	joint_m[5] = DataConfig.JOINT_0029

	self.teensy_io.write('a,2')

	for cnt in range(6):
	    joint_action = joint_m[cnt]
	    if cnt not in [4, 5]: self._movement(joint_action)
	    if cnt in [3]: subprocess.call("sudo python movement_head2.py -m 1 -ns 0.6 -ne 0.6 -nm 0.6 -c 1", shell=True)

	self.teensy_io.write('a,1')

	f = open("position_data.txt", 'w')
	data = "standup"
	f.write(data)
	f.close()

	if int(args.c) == 1: exit()

        for sn in range(18):
            self._disconnect(sn)

        self.pwm.close()
	

    def _movement(self, joint_action):

        servo_sn = 0
        set_value_ss = int(joint_action[3])
        set_value_sa = int(joint_action[4])
        
        for dt in range(10, 28):
            ss = set_value_ss + int(joint_action[dt])
            sa = set_value_sa + int(joint_action[dt + 18])
            target_angle = float(joint_action[dt + 36]) + float(self.joint_gap[dt - 10]) 

            self.pwm.setSpeed(servo_sn, ss)
            self.pwm.setAccel(servo_sn, sa)
            self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle)) 

            servo_sn += 1

        ts = joint_action[5]
        time.sleep(ts)

    def _set_smooth(self):
	target_angle = 0
	for sn in range(18):
	    self.pwm.setSpeed(sn,20)
	    self.pwm.setAccel(sn,5)

    def _disconnect(self, sn):
        self.pwm.setTarget(sn, 0)

    def _convert_radian2pulse(self, radian_value):
        target = int(637 * radian_value + 1500) * 4
        target = np.clip(target, 2000, 10000)
        return target # (pi + 2x) / pi * 1000 + 500

if __name__ == '__main__':
    try:
        mr = MoveACT()
    except KeyboardInterrupt:
        print("Shutting down - MoveACT")
