#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact
'''

import maestro
import time
import random
import numpy as np
import sys, os
sys.path.append('./')
from data_config import DataConfig
from optset_config import OptsetConfig

class MoveACT:

    def __init__(self):

	if self._posture_read() not in ["standup"]: exit()

        self.pwm = maestro.Controller()
        time.sleep(1)

	self.joint_gap = OptsetConfig.JOINT_GAP_0
	self._shake()

    def _posture_read(self):
	f = open("position_data.txt", 'r')
	data = f.readline()
	f.close()
	return data

    def _shake(self):
	joint_sn = [1, 4, 7, 11, 17]
	joint_set_target = [0.1, 0, 0.1, 0]
	joint_r_direction = [0.15, 0.2, 0.15, 0.2]
	joint_l_direction = [-0.15, -0.2, -0.15, -0.2]

	for cnt in range(5):
           self.pwm.setSpeed(joint_sn[cnt],20)
           self.pwm.setAccel(joint_sn[cnt],5)

	shake_target = random.randint(0, 1) # rigth 0, left 1
	shake_step = random.randint(2, 6)
	for step in range(shake_step):
	   if shake_target in [0]:
	      joint_target = np.add(joint_set_target, joint_r_direction)
	      for cnt in range(4):
	         target_angle = joint_target[cnt] + self.joint_gap[joint_sn[cnt]]
     	         self.pwm.setTarget(joint_sn[cnt], self._convert_radian2pulse(target_angle))
              self.pwm.setTarget(17, self._convert_radian2pulse(0.5))
              shake_target = 1
	   else:
	      joint_target = np.add(joint_set_target, joint_l_direction)
	      for cnt in range(4):
	         target_angle = joint_target[cnt] + self.joint_gap[joint_sn[cnt]]
      	         self.pwm.setTarget(joint_sn[cnt], self._convert_radian2pulse(target_angle))
     	      self.pwm.setTarget(17, self._convert_radian2pulse(-0.2))
   	      shake_target = 0
	   time.sleep(1)

	for cnt in range(4):
	   self.pwm.setTarget(joint_sn[cnt], self._convert_radian2pulse(joint_set_target[cnt]))

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
