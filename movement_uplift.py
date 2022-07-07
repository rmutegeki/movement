#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact
'''

import maestro
import time
import numpy as np
import argparse
import sys, os


sys.path.append('./')
from data_config import DataConfig
from optset_config import OptsetConfig

parser = argparse.ArgumentParser(description='')
parser.add_argument('-c', required=True, help='servo active 1, close 0')
parser.add_argument('-x', required=True, help=' 1, 0')
parser.add_argument('-r', required=True, help=' 1, 0')
args = parser.parse_args()

class MoveACT:

    def __init__(self):

	f = open("position_data.txt", 'r')
	data = f.readline()
	f.close()
	if int(args.x) == 0 and data not in ["standup"]: exit()

        self.pwm = maestro.Controller()
        time.sleep(1)

	self.joint_gap = OptsetConfig.JOINT_GAP_0

	joint_m = np.zeros((11, 64))
	joint_m[0] = DataConfig.JOINT_0029
	joint_m[1] = DataConfig.JOINT_0035 # 0035
	joint_m[2] = DataConfig.JOINT_0048
	joint_m[3] = DataConfig.JOINT_0047
	joint_m[4] = DataConfig.JOINT_0046
	joint_m[5] = DataConfig.JOINT_0045
	joint_m[6] = DataConfig.JOINT_0044
	joint_m[7] = DataConfig.JOINT_0043
	joint_m[8] = DataConfig.JOINT_0027 # 0027

	joint_m[9] = DataConfig.JOINT_0035
	joint_m[10] = DataConfig.JOINT_0036

	cn_opt = int(args.r)
	cn = 0
	for cnt in range(9):
	    if cn_opt == 0: cn = cnt
	    else: cn = 8 - cnt 
	    joint_action = joint_m[cn]
	    if cn not in [0, 1]: self._movement(joint_action, self.joint_gap)
#	    print cn 
#	    a = input()

	if cn_opt in [1]:
	   for cnt in range(9, 11):
	      joint_action = joint_m[cnt]
	      self._movement(joint_action, self.joint_gap)
	   self._leg_posture_set()

	f = open("position_data.txt", 'w')
	data = "uplift"
	f.write(data)
	f.close

        if int(args.c) == 1: exit() 

        for sn in range(18):
            self._disconnect(sn)
	
	self.pwm.close()


    def _movement(self, joint_action, joint_gap):

        servo_sn = 0
        set_value_ss = int(joint_action[3])
        set_value_sa = int(joint_action[4])

        for dt in range(10, 28):
            ss = set_value_ss + int(joint_action[dt])
            sa = set_value_sa + int(joint_action[dt + 18])
            target_angle = float(joint_action[dt + 36]) + float(joint_gap[dt - 10])

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
