#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact
'''

import maestro
import time
import math as m
import argparse
import random
import numpy as np
import sys, os
sys.path.append('./')
from optset_config import OptsetConfig

parser = argparse.ArgumentParser(description='Position the head in time direction')
parser.add_argument('-posture', required=False, help='posture sit 0, stand 1')
parser.add_argument('-m', required=False, help='motion type')
parser.add_argument('-ns', required=False, help='neck start position from limit -0.7~0.8')
parser.add_argument('-ne', required=False, help='neck end servo position from limit -0.7~0.8')
parser.add_argument('-nm', required=False, help='up down limit - down:ns ~ ')
parser.add_argument('-delay', required=False, help='delay time')
parser.add_argument('-c', required=False, default=0, help='servo close 0, fix 1')


args = parser.parse_args()

class MoveACT:

    def __init__(self):
        self.pwm = maestro.Controller()
        time.sleep(0.1)
	
	self.joint_gap = OptsetConfig.JOINT_GAP_0

	self.neck_max = 0.8 # up
	self.neck_min = -0.7 # down
	self.neck_servo_sn = 14

	if args.ns: self.neck_start_set = float(args.ns)
	if args.ne: self.neck_end_set = float(args.ne)
	if args.nm: self.neck_limit_set = float(args.nm)
	if args.m: motion_type = int(args.m)
	else: motion_type = random.randint(0, 8)

	if args.posture in ["0"]: self.neck_start_set = -0.4; self.neck_end_set = -0.4; self.neck_limit_set = 0.3
	if args.posture in ["1"]: self.neck_start_set = 0.3; self.neck_end_set = 0.5; self.neck_limit_set = 0.5

	self._set_start()
	if motion_type == 0: self._m_updown()
	if motion_type == 1: self._m_wiggle(0.6)
	if motion_type == 2: self._forward_check(0.2)
	if motion_type == 3: self._nodding_right(0.2)
	if motion_type == 4: self._nodding_left(0.2)
	if motion_type == 5: self._nodding_front(0.2)
	if motion_type == 6: self._sit_up()
	if motion_type == 7: self._sit_right_up()
	if motion_type == 8: self._sit_left_up()

	self._set_end()
        
        if int(args.c) == 0: self._disconnect()

        self.pwm.close()

    def _set_start(self):
	joint_action = [ 100, 20, 1, 0, 0, -1 ]
	joint_action[3] = self.neck_start_set
	self._movement_head(joint_action)

    def _set_end(self):
	joint_action = [ 20, 5, 1, 0, 0, -1 ]
	joint_action[3] = self.neck_end_set
	self._movement_head(joint_action)

    def _sit_up(self):
	joint_action = [ 20, 5, 1, 0, 0, -0.8 ]
	joint_action[3] = self.neck_start_set
	self._movement_head(joint_action)


	joint_action = [ 20, 5, 3, 0, 0, -1.3 ]
	if args.delay: joint_action[2] = float(args.delay)
	joint_action[3] = self.neck_start_set + self.neck_limit_set 
	self._movement_head(joint_action)

    def _sit_right_up(self):
	joint_action = [ 20, 5, 1, 0, 0, -0.8 ]
	joint_action[3] = self.neck_start_set
	self._movement_head(joint_action)

	
	joint_action = [ 20, 5, 3, 0, -0.6, -1.3 ]
	if args.delay: joint_action[2] = float(args.delay)
	joint_action[3] = self.neck_start_set + self.neck_limit_set
	self._movement_head(joint_action)

    def _sit_left_up(self):
	joint_action = [ 20, 5, 1, 0, 0, -0.8 ]
	joint_action[3] = self.neck_start_set
	self._movement_head(joint_action)

	
	joint_action = [ 20, 5, 3, 0, 0.6, -1.3 ]
	if args.delay: joint_action[2] = float(args.delay)
	joint_action[3] = self.neck_start_set + self.neck_limit_set 
	self._movement_head(joint_action)


    def _m_updown(self):
	joint_action = [ 20, 5, 0.5, 0, 0, -1 ]
	joint_action[3] = self.neck_start_set
	self._movement_head(joint_action)

	joint_action = [ 50, 5, 1.5, 0, 0, -1.3 ]
	joint_action[3] = self.neck_start_set + self.neck_limit_set
	self._movement_head(joint_action)

	joint_action = [ 50, 5, 2, 0, 0, -0.6 ]
	joint_action[3] = self.neck_start_set
  	self._movement_head(joint_action)

    def _m_wiggle(self, hs):
	ns = self.neck_start_set
	joint_action = [ 20, 30, 0.1, 0, 0, -1 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
  	self._movement_head(joint_action)

	joint_action = [ 0, 30, 0.5, 0, -1, -1.2]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	joint_action = [ 0, 30, 0.5, 0, 1, -1.35]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	joint_action = [ 250, 0, 0.3, 0, -0.7, -1.2]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	joint_action = [ 0, 0, 0.3, 0, 0.7, -1.3 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	joint_action = [ 20, 5, 0.1, 0, 0, -1 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

    def _forward_check(self, hs):
	ns = self.neck_start_set
	joint_action = [ 20, 30, 0.3, 0, 0, -1 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
  	self._movement_head(joint_action)

	joint_action = [ 150, 30, 0.8, 0, -0.8, -1]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	joint_action = [ 30, 10, 1.5, 0, 0.8, -1]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	joint_action = [ 20, 30, 0.5, 0, 0, -1 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
  	self._movement_head(joint_action)

    def _nodding_right(self, hs):
	ns = self.neck_start_set
	joint_action = [ 20, 30, 0.3, 0, 0, -1 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
  	self._movement_head(joint_action)

	joint_action = [ 150, 30, 0.8, 0, -0.6, -1]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	for cnt in range(2):
	   joint_action = [ 30, 20, 0.5, 0, -0.6, -1.4]
	   joint_action[3] = ns
	   joint_action[5] = joint_action[5] + hs
	   self._movement_head(joint_action)

	   joint_action = [ 30, 20, 0.5, 0, -0.6, -1 ]
 	   joint_action[3] = ns
	   joint_action[5] = joint_action[5] + hs
  	   self._movement_head(joint_action)

	joint_action = [ 30, 20, 0.5, 0, -0.6, -1.4]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

    def _nodding_left(self, hs):
	ns = self.neck_start_set
	joint_action = [ 20, 30, 0.3, 0, 0, -1 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
  	self._movement_head(joint_action)

	joint_action = [ 150, 30, 0.8, 0, 0.6, -1]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	for cnt in range(2):
	   joint_action = [ 30, 20, 0.5, 0, 0.6, -1.4]
	   joint_action[3] = ns
	   joint_action[5] = joint_action[5] + hs
	   self._movement_head(joint_action)

	   joint_action = [ 30, 20, 0.5, 0, 0.6, -1 ]
 	   joint_action[3] = ns
	   joint_action[5] = joint_action[5] + hs
  	   self._movement_head(joint_action)

	joint_action = [ 30, 20, 0.5, 0, 0.6, -1.4]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

    def _nodding_front(self, hs):
	ns = self.neck_start_set
	joint_action = [ 20, 30, 0.3, 0, 0, -1 ]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
  	self._movement_head(joint_action)

	joint_action = [ 150, 30, 0.8, 0, 0, -1]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)

	for cnt in range(3):
	   joint_action = [ 20, 5, 0.7, 0, 0, -1.4]
	   joint_action[3] = ns
	   joint_action[5] = joint_action[5] + hs
	   self._movement_head(joint_action)

	   joint_action = [ 30, 20, 0.7, 0, 0, -1 ]
 	   joint_action[3] = ns
	   joint_action[5] = joint_action[5] + hs
  	   self._movement_head(joint_action)

	joint_action = [ 30, 20, 0.5, 0, 0, -1.4]
	joint_action[3] = ns
	joint_action[5] = joint_action[5] + hs
	self._movement_head(joint_action)


    def _movement_head(self, joint_action):

        ss = joint_action[0]
        sa = joint_action[1]

	if joint_action[3] < self.neck_min: joint_action[3] = self.neck_min
	if joint_action[3] > self.neck_max: joint_action[3] = self.neck_max

	head_sn = 3
        for sn in range(14, 17):
            target_angle = joint_action[head_sn] + self.joint_gap[sn]
            self.pwm.setSpeed(sn, ss)
            self.pwm.setAccel(sn, sa)
            self.pwm.setTarget(sn, self._convert_radian2pulse(target_angle))
            head_sn += 1

        ts = joint_action[2]
        time.sleep(ts)

    def _disconnect(self):
        for sn in range(14, 17):
            self.pwm.setTarget(sn, 0)

    def _convert_radian2pulse(self, radian_value):
        target = int(637 * radian_value + 1500) * 4
        target = np.clip(target, 2000, 10000)
        return target # (pi + 2x) / pi * 1000 + 500

#    def _avg(self, *args):
#        result = 0
#        for i_val in args:
#            result += i_val
#        return result / len(args)


if __name__ == '__main__':
    try:
        mr = MoveACT()
    except KeyboardInterrupt:
        print("Shutting down - MoveACT")
