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
import FaBo9Axis_MPU9250

sys.path.append('./')
from data_config import DataConfig


parser = argparse.ArgumentParser(description='')
parser.add_argument('-c', required=True, help='servo active 1, close 0')
args = parser.parse_args()

class MoveACT:

    def __init__(self):

	mpu9250 = FaBo9Axis_MPU9250.MPU9250()
	time.sleep(1)
	pos_Y = 0
	count_Y = 10
        self.falldown = 0
	for cnt in range(count_Y):
	   Accel = mpu9250.readAccel()
           pos_Y += Accel['y'] 
	   time.sleep(0.05)
	pos_Y = round((pos_Y / count_Y), 1)
	if pos_Y > 0.5: self.falldown = 1   # right
	elif pos_Y < -0.5 : self.falldown = -1 # left
        elif pos_Y == 0 : self.falldown = 1 # center
	else: 
	   print pos_Y
    	   quit()

        self.pwm = maestro.Controller()
        time.sleep(1)

	self.joint_gap = DataConfig.JOINT_GAP_0

#	joint_action = DataConfig.JOINT_0029
#	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0040
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0041
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0042
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0043
	self._movement(joint_action, self.joint_gap)

	self._act_uplift()

	self._act_sitdown2()

	f = open("position_data.txt", 'w')
	data = "sitdown2"
	f.write(data)
	f.close()

	if int(args.c) == 1: exit()

        for sn in range(18):
            self._disconnect(sn)

        self.pwm.close()

    def _act_standup(self):

	joint_action = DataConfig.JOINT_0027
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0028
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0029
	self._movement(joint_action, self.joint_gap)

    def _act_uplift(self):

	joint_action = DataConfig.JOINT_0044
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0045
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0046
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0047
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0048
	self._movement(joint_action, self.joint_gap)

    def _act_sitdown2(self):

	joint_action = DataConfig.JOINT_0035
	self._movement(joint_action, self.joint_gap)

	joint_action = DataConfig.JOINT_0036
	self._movement(joint_action, self.joint_gap)


    def _movement(self, joint_action, joint_gap):

        servo_sn = 0
        set_value_ss = joint_action[3]
        set_value_sa = joint_action[4]
        servo_sn_r_list = [3, 4, 5, 0, 1, 2, 10, 11, 12, 13, 6, 7, 8, 9]
        for dt in range(10, 28):
            ss = set_value_ss + joint_action[dt]
            sa = set_value_sa + joint_action[dt + 18]
            target_angle = joint_action[dt + 36] + joint_gap[dt - 10]

            self.pwm.setSpeed(servo_sn, ss)
            self.pwm.setAccel(servo_sn, sa)
	    if self.falldown == 1:
            	self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle)) 
	    elif servo_sn in [14, 15, 16, 17]:
                self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle))
	    else:
		servo_sn_r = servo_sn_r_list[servo_sn]
		target_angle_r = -1 * joint_action[dt + 36]
                target_angle_r = target_angle_r + joint_gap[servo_sn_r]
		self.pwm.setTarget(servo_sn_r, self._convert_radian2pulse(target_angle_r))
#                print servo_sn_r, target_angle, target_angle_r
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
#        target = np.clip(target, 2000, 10000)

        return target # (pi + 2x) / pi * 1000 + 500

if __name__ == '__main__':
    try:
        mr = MoveACT()
    except KeyboardInterrupt:
        print("Shutting down - MoveACT")
