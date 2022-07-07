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
args = parser.parse_args()

class MoveACT:

    def __init__(self):
        self.pwm = maestro.Controller()
        time.sleep(1)

	self.joint_gap = DataConfig.JOINT_GAP_0

	joint_action = DataConfig.JOINT_0029
	self._movement(joint_action, self.joint_gap)

	for sn in range(4):
#    	    joint_action = DataConfig.JOINT_0060
#	    self._movement(joint_action, self.joint_gap)

  	    self._set_smooth(0, 0)
	    self._turn()

    	    joint_action = DataConfig.JOINT_0029
 	    self._movement(joint_action, self.joint_gap)

	if int(args.c) == 1: exit()

        for sn in range(18):
            self._disconnect(sn)

        self.pwm.close()

    def _turn(self):
	
	sstp = 0.2
	delay = 0.3
	joint_delay = 0.3
	joint_action = [ 0, 0.1, -1.5,   0, -0.1, 1.5,   -0.2, 0, 0.5, -0.5,   0.2, 0, -0.5, 0.5 ]

	joint_action[3] = -0.4
	joint_action[5] = 0.4
	self._turn_act(joint_delay, joint_action, self.joint_gap)
	joint_action[4] += sstp 
	self._turn_act(delay, joint_action, self.joint_gap)
	joint_action[3] = 0
	joint_action[5] = 1.5
	self._turn_act(joint_delay, joint_action, self.joint_gap)


	joint_action[6] = -0.9
	joint_action[8] = -0.3
        joint_action[9] = -0.9
	self._turn_act(joint_delay, joint_action, self.joint_gap)
	joint_action[7] -= sstp 
	self._turn_act(delay, joint_action, self.joint_gap)
	joint_action[6] = -0.2
	joint_action[8] = 0.5
        joint_action[9] = -0.5
	self._turn_act(joint_delay, joint_action, self.joint_gap)


	joint_action[1] -= sstp
	joint_action[4] -= sstp 
	self._turn_act(delay, joint_action, self.joint_gap)
	joint_action[0] = 0.4
	joint_action[2] = -0.4 
	self._turn_act(joint_delay, joint_action, self.joint_gap)
	joint_action[1] += sstp 
	self._turn_act(delay, joint_action, self.joint_gap)
	joint_action[0] = 0
	joint_action[2] = -1.5
	self._turn_act(joint_delay, joint_action, self.joint_gap)


	joint_action[7] += sstp
	joint_action[11] += sstp 
	self._turn_act(delay, joint_action, self.joint_gap)
	joint_action[10] = 0.9
	joint_action[12] = 0.3
        joint_action[13] = 0.9
	self._turn_act(joint_delay, joint_action, self.joint_gap)
	joint_action[11] -= sstp 
	self._turn_act(delay, joint_action, self.joint_gap)
	joint_action[10] = 0.2
	joint_action[12] = -0.5
        joint_action[13] = 0.5
	self._turn_act(joint_delay, joint_action, self.joint_gap)

    def _turn_act(self, slt, joint_action, joint_gap):
	servo_sn = 0
        for dt in range(14):
            target_angle = joint_action[dt] + joint_gap[servo_sn]
            self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle)) 
            servo_sn += 1
	time.sleep(slt)

#	a = input()

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
