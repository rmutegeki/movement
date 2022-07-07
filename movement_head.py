#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact
'''

import maestro
import time
import math as m
import argparse

import numpy as np
import sys, os
sys.path.append('./')
# from data_config import DataConfig

parser = argparse.ArgumentParser(description='Position the head in time direction')
parser.add_argument('-t', required=True, help='target angle')
args = parser.parse_args()

class MoveACT:

    def __init__(self):
        self.pwm = maestro.Controller()
        time.sleep(0.1)
 

        neck_lift_mid = self._avg(-0.7, 0.7)
        neck_pen_mid = self._avg(-1, 1)
	head_lift_mid = self._avg(-1.5, -0.7)

	self._set_position(neck_lift_mid, neck_pen_mid, head_lift_mid)

        tn = int(args.t)
        self._motion(tn, neck_lift_mid, neck_pen_mid, head_lift_mid)    

  	self._set_position(neck_lift_mid, neck_pen_mid, head_lift_mid)

        for sn in range(18):
            self._disconnect(sn)

        self.pwm.close()

    def _set_position(self, nlm, npm, hlm):
	joint_action = [ 120, 20, 0.5, nlm, npm, hlm ]
	self._movement(joint_action)


    def _motion(self, target_time, nl_ctr, np_ctr, hl_ctr):
        set_angle = (360 / 12 ) * target_time
        target_radian =  m.radians(set_angle)

        xp = 0.6 * m.sin(target_radian)
        yn = 0.4 + ( 0.7 * m.cos(target_radian) )
        yh = -1.5 + ( -0.55 * m.sin(target_radian) )

        y_nl = yn
        xx   = xp
        y_hl = yh

	joint_action = [ 150, 40, 1, y_nl, xx, y_hl ]
	self._movement(joint_action)

#	print target_time, joint_action

    def _movement(self, joint_action):

        set_value_ss = joint_action[0]
        set_value_sa = joint_action[1]

	head_sn = 3
        for sn in range(14, 17):

            ss = set_value_ss
            sa = set_value_sa
            target_angle = joint_action[head_sn]

            self.pwm.setSpeed(sn, ss)
            self.pwm.setAccel(sn, sa)
            self.pwm.setTarget(sn, self._convert_radian2pulse(target_angle))
            head_sn += 1

        ts = joint_action[2]
        time.sleep(ts)

    def _set_smooth(self):
	target_angle = 0
	for sn in range(14, 16):
	    self.pwm.setSpeed(sn,20)
	    self.pwm.setAccel(sn,5)

    def _disconnect(self, sn):
        self.pwm.setTarget(sn, 0)

    def _convert_radian2pulse(self, radian_value):
        target = int(637 * radian_value + 1500) * 4
        target = np.clip(target, 3000, 9000)
        return target # (pi + 2x) / pi * 1000 + 500

    def _avg(self, *args):
        result = 0
        for i_val in args:
            result += i_val
        return result / len(args)


if __name__ == '__main__':
    try:
        mr = MoveACT()
    except KeyboardInterrupt:
        print("Shutting down - MoveACT")
