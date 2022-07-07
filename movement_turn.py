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


parser = argparse.ArgumentParser(description='')
parser.add_argument('-c', required=True, help='servo active 1, close 0')
parser.add_argument('-st', required=True, help='step')
parser.add_argument('-tn', required=True, help='right turn 1, left turn 0')

args = parser.parse_args()

class MoveACT:

    def __init__(self):
        self.pwm = maestro.Controller()
        time.sleep(1)

	self.joint_gap = DataConfig.JOINT_GAP_0

	joint_action = DataConfig.JOINT_0029
	self._movement(joint_action, self.joint_gap)

	step = int(args.st)
	dir = int(args.tn)
	for sn in range(step):
    	    joint_action = DataConfig.JOINT_0060
	    if dir == 1: joint_action = DataConfig.JOINT_0062
	    self._movement(joint_action, self.joint_gap)
    	    self._set_smooth(0, 40)
	    self._turn(dir)
#    	    joint_action = DataConfig.JOINT_0061
#	    if dir == 1: joint_action = DataConfig.JOINT_0063
# 	    self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0029
	self._movement(joint_action, self.joint_gap)

	if int(args.c) == 1: exit()

        for sn in range(18):
            self._disconnect(sn)

        self.pwm.close()

    def _turn(self, dir):

	rev_f = [-1, -1, -1]
	rev_b = [-1, -1, -1, -1]

	if dir == 0: 
     	   servo_st = [ 3, 6, 0, 10]
	else: servo_st = [0, 10, 3, 6]
	
	joint_action = [-0.3, 0.25, 0.7]
	if dir == 1: joint_action = np.multiply(joint_action, rev_f)
	self._turn_f(servo_st[0], joint_action, self.joint_gap)

	joint_action = [-0.3, -0.15, 0.7]
	if dir == 1: joint_action = np.multiply(joint_action, rev_f)
	self._turn_f(servo_st[0], joint_action, self.joint_gap)

	joint_action = [0, -0.15, 1.5]
	if dir == 1: joint_action = np.multiply(joint_action, rev_f)
	self._turn_f(servo_st[0], joint_action, self.joint_gap)



	joint_action = [-0.9, -0.2, -0.2, -1.1]
	if dir == 1: joint_action = np.multiply(joint_action, rev_b)
	self._turn_b(servo_st[1], joint_action, self.joint_gap)

	joint_action = [-0.9, 0.1, -0.2, -1.1]
	if dir == 1: joint_action = np.multiply(joint_action, rev_b)
	self._turn_b(servo_st[1], joint_action, self.joint_gap)

	joint_action = [-0.2, 0.1, 0.5, -0.5]
	if dir == 1: joint_action = np.multiply(joint_action, rev_b)
	self._turn_b(servo_st[1], joint_action, self.joint_gap)



	joint_action = [0.3, 0.25, -0.7]
	if dir == 1: joint_action = np.multiply(joint_action, rev_f)
	self._turn_f(servo_st[2], joint_action, self.joint_gap)

	joint_action = [0.3, -0.15, -0.7]
	if dir == 1: joint_action = np.multiply(joint_action, rev_f)
	self._turn_f(servo_st[2], joint_action, self.joint_gap)

	joint_action = [0, -0.15, -1.5]
	if dir == 1: joint_action = np.multiply(joint_action, rev_f)
	self._turn_f(servo_st[2], joint_action, self.joint_gap)


	joint_action = [0.9, -0.2, 0.2, 1.1]
	if dir == 1: joint_action = np.multiply(joint_action, rev_b)
	self._turn_b(servo_st[3], joint_action, self.joint_gap)

	joint_action = [0.9, 0.1, 0.2, 1.1]
	if dir == 1: joint_action = np.multiply(joint_action, rev_b)
	self._turn_b(servo_st[3], joint_action, self.joint_gap)

	joint_action = [0.2, 0.1, -0.5, 0.5]
	if dir == 1: joint_action = np.multiply(joint_action, rev_b)
	self._turn_b(servo_st[3], joint_action, self.joint_gap)
	
    def _turn_f(self, servo_sn, joint_action, joint_gap):
#       right front leg
        for dt in range(3):
            target_angle = joint_action[dt] + joint_gap[servo_sn]
            self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle)) 
            servo_sn += 1
	time.sleep(0.2)

    def _turn_b(self, servo_sn, joint_action, joint_gap):
#       right front leg
        for dt in range(4):
            target_angle = joint_action[dt] + joint_gap[servo_sn]
            self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle)) 
            servo_sn += 1
        time.sleep(0.3)

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

    def _set_smooth(self, ss, sa):
	target_angle = 0
	for sn in range(18):
	    self.pwm.setSpeed(sn,ss)
	    self.pwm.setAccel(sn,sa)

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
