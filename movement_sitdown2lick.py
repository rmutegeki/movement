#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact
'''

import maestro
import time
import numpy as np
import argparse
import subprocess
import random
import sys, os

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

	self.joint_gap = OptsetConfig.JOINT_GAP_0

	joint_m = np.zeros((5, 64))
	joint_m[0] = DataConfig.JOINT_0036
	joint_m[1] = DataConfig.JOINT_0075 # right
	joint_m[2] = DataConfig.JOINT_0076
	joint_m[3] = DataConfig.JOINT_0077 # left
	joint_m[4] = DataConfig.JOINT_0078


	self._movement(joint_m[0])

	subprocess.call("sudo python cmd_headaction.py -a 3", shell=True)

	sw = random.randint(1,1)
	if sw == 2: sw = 3
	for cnt in range(3):
	   self._movement(joint_m[sw])
	   self._movement(joint_m[1 + sw])

	subprocess.call("sudo python cmd_headaction.py -a 1", shell=True)

	self._movement(joint_m[0])

	self._leg_posture_set()

	subprocess.call("sudo python movement_head2.py -m 2 -ns -0.5 -ne -0.5 -nm 0 -c 1", shell=True)

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

    def _leg_posture_set(self):
        self._set_smooth_limit([0,5,70,20]) # st, en, speed, accel
        self._leg_movement([3, 5, 0.6, -1.3, -0.2, 0.7]) # st, en, sleep, servo_sn....
        self._leg_movement([3, 5, 0.7, -0.7, -0.25, 1.5]) # st, en, sleep, servo_sn....
        self._leg_movement([0, 2, 0.6, 1.3, 0.2, -0.7]) # st, en, sleep, servo_sn....
        self._leg_movement([0, 2, 0.7, 0.7, 0.25, -1.5]) # st, en, sleep, servo_sn....


    def _leg_movement(self, data):
        target_sn = 3
        for sn in range(data[0], data[1]+1):
            target_angle = data[target_sn] + self.joint_gap[sn]
            self.pwm.setTarget(sn, self._convert_radian2pulse(target_angle))
            target_sn += 1
        ts = data[2]
        time.sleep(ts)

    def _set_smooth_limit(self, data):
        for sn in range(data[0], data[1]):
            self.pwm.setSpeed(sn, data[2])
            self.pwm.setAccel(sn, data[3])

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
