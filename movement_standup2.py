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
parser.add_argument('-x', required=True, help='1, 0')

args = parser.parse_args()

class MoveACT:

    def __init__(self):

	f = open("position_data.txt", 'r')
	data = f.readline()
	f.close()
	if int(args.x) == 0 and data not in ["sitdown", "sitdown2"]: exit()

        self.pwm = maestro.Controller()
        time.sleep(1)

	self.joint_gap = OptsetConfig.JOINT_GAP_0

# standup2

	joint_action = DataConfig.JOINT_0036
	self._movement(joint_action, self.joint_gap)
	
	joint_action = DataConfig.JOINT_0038
	self._movement(joint_action, self.joint_gap)
#	a = input()
	joint_action = DataConfig.JOINT_0039
	self._movement(joint_action, self.joint_gap)
#	a = input()
	joint_action = DataConfig.JOINT_0029
	self._movement(joint_action, self.joint_gap)


	f = open("position_data.txt", 'w')
	data = "standup"
	f.write(data)
	f.close

	if int(args.c) == 1: exit()	

        for sn in range(18):
            self._disconnect(sn)

        self.pwm.close()

    def _movement(self, joint_action, joint_gap):

        servo_sn = 0
        set_value_ss = joint_action[3]
        set_value_sa = joint_action[4]
        
        for dt in range(10, 28):
            ss = set_value_ss + joint_action[dt]
            sa = set_value_sa + joint_action[dt + 18]
            target_angle = joint_action[dt + 36] + joint_gap[dt - 10]

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
