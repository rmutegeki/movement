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
#import FaBo9Axis_MPU9250

sys.path.append('./')
from data_config import DataConfig

parser = argparse.ArgumentParser(description='')
parser.add_argument('-st', required=True, help='Walking step')
parser.add_argument('-ws', required=True, help='Walking speed')
parser.add_argument('-c', required=True, help='servo active 1, close 0')
args = parser.parse_args()

class MoveACT:

    def __init__(self):

	f = open("position_data.txt", 'r')
	data = f.readline()
	f.close
#	if data not in ["standup"]: exit()

        walking_step = int(args.st)

        self.pwm = maestro.Controller()
        time.sleep(1)

        # Walk
        self._motor_direction = [1, 1, -1, -1,
                                 -1, -1, 1, 1,
                                 1, 1,
                                 1, 1, -1,
                                 -1, -1, 1]
        self.trajs = np.zeros((46, 16))
        self.trajs[0] = [-0.237,0.000,0.549,-1.098,-0.687,-0.000,0.276,-0.717,0,0,-0.357,-0.000,-0.001,0.376,-0.000,-0.612]
        self.trajs[1] = [-0.208,0.000,0.540,-1.085,-0.679,-0.000,0.309,-0.762,0,0,-0.339,-0.000,-0.001,0.390,-0.000,-0.597]
        self.trajs[2] = [-0.179,0.000,0.530,-1.069,-0.670,-0.000,0.340,-0.804,0,0,-0.321,-0.000,-0.001,0.402,-0.000,-0.580]
        self.trajs[3] = [-0.149,0.000,0.517,-1.052,-0.659,-0.000,0.368,-0.843,0,0,-0.206,-0.000,-0.212,0.413,-0.000,-0.559]
        self.trajs[4] = [-0.094,-0.000,0.536,-1.078,-0.647,-0.000,0.395,-0.880,0,0,-0.147,-0.000,-0.301,0.487,-0.000,-0.612]
        self.trajs[5] = [-0.073,-0.000,0.611,-1.119,-0.634,-0.000,0.419,-0.913,0,0,-0.098,-0.000,-0.365,0.592,-0.000,-0.759]
        self.trajs[6] = [-0.071,-0.000,0.695,-1.119,-0.620,-0.000,0.441,-0.944,0,0,-0.056,-0.000,-0.416,0.679,-0.000,-0.898]
        self.trajs[7] = [-0.089,-0.000,0.775,-1.119,-0.604,-0.000,0.462,-0.973,0,0,-0.017,-0.000,-0.458,0.738,-0.000,-1.013]
        self.trajs[8] = [-0.129,-0.000,0.848,-1.119,-0.588,-0.000,0.480,-0.999,0,0,0.018,-0.000,-0.494,0.768,-0.000,-1.109]
        self.trajs[9] = [-0.194,-0.000,0.916,-1.119,-0.570,-0.000,0.497,-1.023,0,0,0.052,-0.000,-0.524,0.769,-0.000,-1.190]
        self.trajs[10] = [-0.281,-0.000,0.976,-1.119,-0.552,-0.000,0.512,-1.044,0,0,0.083,-0.000,-0.550,0.744,-0.000,-1.256]
        self.trajs[11] = [-0.385,-0.000,0.999,-1.119,-0.532,-0.000,0.525,-1.063,0,0,0.113,-0.000,-0.572,0.697,-0.000,-1.305]
        self.trajs[12] = [-0.497,-0.000,0.999,-1.119,-0.512,-0.000,0.536,-1.079,0,0,0.142,-0.000,-0.591,0.631,-0.000,-1.337]
        self.trajs[13] = [-0.611,-0.000,0.999,-1.119,-0.491,-0.000,0.546,-1.093,0,0,0.169,-0.000,-0.606,0.552,-0.000,-1.348]
        self.trajs[14] = [-0.720,-0.000,0.999,-1.119,-0.468,-0.000,0.554,-1.104,0,0,0.195,-0.000,-0.619,0.462,-0.000,-1.338]
        self.trajs[15] = [-0.817,-0.000,0.999,-1.119,-0.445,-0.000,0.560,-1.113,0,0,0.220,-0.000,-0.629,0.363,-0.000,-1.305]
        self.trajs[16] = [-0.895,-0.000,0.947,-1.119,-0.422,-0.000,0.565,-1.119,0,0,0.244,-0.000,-0.636,0.257,-0.000,-1.243]
        self.trajs[17] = [-0.948,-0.000,0.856,-1.119,-0.397,-0.000,0.568,-1.119,0,0,0.266,-0.000,-0.640,0.147,-0.000,-1.149]
        self.trajs[18] = [-0.968,-0.000,0.742,-1.119,-0.372,-0.000,0.569,-1.119,0,0,0.287,-0.000,-0.642,0.035,-0.000,-1.017]
        self.trajs[19] = [-0.952,-0.000,0.612,-1.119,-0.346,-0.000,0.568,-1.119,0,0,0.307,-0.000,-0.641,-0.073,-0.000,-0.847]
        self.trajs[20] = [-0.902,-0.000,0.477,-0.994,-0.320,-0.000,0.566,-1.119,0,0,0.326,-0.000,-0.638,-0.177,-0.000,-0.630]
        self.trajs[21] = [-0.826,-0.000,0.352,-0.822,-0.293,-0.000,0.562,-1.116,0,0,0.344,-0.000,-0.632,-0.294,-0.000,-0.330]
        self.trajs[22] = [-0.741,-0.000,0.264,-0.701,-0.265,-0.000,0.557,-1.108,0,0,0.361,-0.000,-0.623,-0.406,-0.000,-0.001]
        self.trajs[23] = [-0.687,0.000,0.276,-0.717,-0.237,-0.000,0.549,-1.098,0,0,0.376,-0.000,-0.612,-0.357,-0.000,-0.001]
        self.trajs[24] = [-0.679,0.000,0.309,-0.762,-0.208,-0.000,0.540,-1.085,0,0,0.390,-0.000,-0.597,-0.339,-0.000,-0.001]
        self.trajs[25] = [-0.670,0.000,0.340,-0.804,-0.179,-0.000,0.530,-1.069,0,0,0.402,-0.000,-0.580,-0.321,-0.000,-0.001]
        self.trajs[26] = [-0.659,0.000,0.368,-0.843,-0.149,-0.000,0.517,-1.052,0,0,0.413,-0.000,-0.559,-0.206,-0.000,-0.212]
        self.trajs[27] = [-0.647,0.000,0.395,-0.880,-0.094,-0.000,0.536,-1.078,0,0,0.487,-0.000,-0.612,-0.147,-0.000,-0.301]
        self.trajs[28] = [-0.634,0.000,0.419,-0.913,-0.073,-0.000,0.611,-1.119,0,0,0.592,-0.000,-0.759,-0.098,-0.000,-0.365]
        self.trajs[29] = [-0.620,0.000,0.441,-0.944,-0.071,-0.000,0.695,-1.119,0,0,0.679,-0.000,-0.898,-0.056,-0.000,-0.416]
        self.trajs[30] = [-0.604,0.000,0.462,-0.973,-0.089,-0.000,0.775,-1.119,0,0,0.738,-0.000,-1.013,-0.017,-0.000,-0.458]
        self.trajs[31] = [-0.588,0.000,0.480,-0.999,-0.129,-0.000,0.848,-1.119,0,0,0.768,-0.000,-1.109,0.018,-0.000,-0.494]
        self.trajs[32] = [-0.570,0.000,0.497,-1.023,-0.194,-0.000,0.916,-1.119,0,0,0.769,0.000,-1.190,0.052,-0.000,-0.524]
        self.trajs[33] = [-0.552,0.000,0.512,-1.044,-0.281,-0.000,0.976,-1.119,0,0,0.744,-0.000,-1.256,0.083,-0.000,-0.550]
        self.trajs[34] = [-0.532,0.000,0.525,-1.063,-0.385,-0.000,0.999,-1.119,0,0,0.697,-0.000,-1.305,0.113,-0.000,-0.572]
        self.trajs[35] = [-0.512,0.000,0.536,-1.079,-0.497,-0.000,0.999,-1.119,0,0,0.631,-0.000,-1.337,0.142,-0.000,-0.591]
        self.trajs[36] = [-0.491,0.000,0.546,-1.093,-0.611,-0.000,0.999,-1.119,0,0,0.552,-0.000,-1.348,0.169,-0.000,-0.606]
        self.trajs[37] = [-0.468,0.000,0.554,-1.104,-0.720,-0.000,0.999,-1.119,0,0,0.462,-0.000,-1.338,0.195,-0.000,-0.619]
        self.trajs[38] = [-0.445,0.000,0.560,-1.113,-0.817,-0.000,0.999,-1.119,0,0,0.363,-0.000,-1.305,0.220,-0.000,-0.629]
        self.trajs[39] = [-0.422,0.000,0.565,-1.119,-0.895,-0.000,0.947,-1.119,0,0,0.257,-0.000,-1.243,0.244,-0.000,-0.636]
        self.trajs[40] = [-0.397,0.000,0.568,-1.119,-0.948,-0.000,0.856,-1.119,0,0,0.147,-0.000,-1.149,0.266,-0.000,-0.640]
        self.trajs[41] = [-0.372,0.000,0.569,-1.119,-0.968,-0.000,0.742,-1.119,0,0,0.035,-0.000,-1.017,0.287,-0.000,-0.642]
        self.trajs[42] = [-0.346,0.000,0.568,-1.119,-0.952,-0.000,0.612,-1.119,0,0,-0.073,-0.000,-0.847,0.307,-0.000,-0.641]
        self.trajs[43] = [-0.320,0.000,0.566,-1.119,-0.902,-0.000,0.477,-0.994,0,0,-0.177,-0.000,-0.630,0.326,-0.000,-0.638]
        self.trajs[44] = [-0.293,0.000,0.562,-1.116,-0.826,-0.000,0.352,-0.822,0,0,-0.294,-0.000,-0.330,0.344,-0.000,-0.632]
        self.trajs[45] = [-0.265,0.000,0.557,-1.108,-0.741,-0.000,0.264,-0.701,0,0,-0.406,-0.000,-0.001,0.361,-0.000,-0.623]
        self.trajs = np.multiply(self.trajs, self._motor_direction)

#       self.stand_set = [-0.4, 0, 0.88, -1.12,  0.4, 0, -0.88, 1.12,    0, 0,   0.1,0.0,-0.4,  -0.1,0.0,0.4]
#       self.stand_set = np.multiply(self.stand_set, self._motor_direction)

#        self.optset = [ 0.9,0.1,1.28,-1.62,   -0.1,-0.1,0.48,-0.62,   0,0,   -0.2,0.1,-1.9,    0,-0.1,1.1 ]
#        self.optset = [ 0.3, 0.11, 1.05, -1.8,   -0.3, -0.1, -1.05, 1.8,   0,0,   -0.2,0.1,-1.9,    0.2,-0.1,1.9 ]
        self.optset = [ 0.3, 0.11, 1.05, -1.8,   -0.3, -0.1, -1.05, 1.8,   0,0,   -0.2,0.1,-1.9,    -0.25, -0.1,1.9 ]

#        self.optset = [ 0,0,0,0,   0,0,0,0,   0,0,   0,0,0,    0,0,0.0 ]

        self.joint_gap = DataConfig.JOINT_GAP_0

        joint_action = DataConfig.JOINT_0029
        self._movement(joint_action, self.joint_gap)

        joint_action = DataConfig.JOINT_backwalk
        self._movement(joint_action, self.joint_gap)

        for sn in range(18):
            self.pwm.setSpeed(sn, 150)
            self.pwm.setAccel(sn, 30)
 
	self._start_walkstep()

        for cn in range(walking_step):
            self._walkstep()
#	    self._gyroCheck()

	self._end_walkstep()

        time.sleep(0.5)

        joint_action = DataConfig.JOINT_0029
        self._movement(joint_action, self.joint_gap)


	if int(args.c) == 1: exit()

        for sn in range(18):
            self._disconnect(sn)

        self.pwm.close()

#    def _gyroCheck(self):
#	mpu9250 = FaBo9Axis_MPU9250.MPU9250()
#	pos_Y = 0
#        count_Y = 1
#	for cnt in range(count_Y):
#	   Accel = mpu9250.readAccel()
#	   pos_Y += Accel['y']
#	pos_Y = round((pos_Y / count_Y), 1)
#	if pos_Y > 0.7 or pos_Y < -0.7: 
#	   if int(args.c) == 1: exit()
#	   else: 
#	      for sn in range(18):
#	          self._disconnect(sn)
#	      exit() 

    def _end_walkstep(self): 
	start = 45
	step = 1
        for cnv in range(10):
	   step -= 1
	   if step == 0:
              self._walkingdata_cnv(start - cnv)
	      step = int(args.ws)
#	      step = 1
	      time.sleep(0.1)

    def _start_walkstep(self): 
	start = 16
	step = 1
        for cnv in range(16):
            step -= 1
            if step == 0 : 
                self._walkingdata_cnv(start - cnv)
		step = int(args.ws)

    def _walkstep(self): 
        step = 1
        for cnv in range(46):
            step -= 1
            if step == 0 : 
#                if cnv in[0, 3, 7, 10, 14, 18, 23, 26, 30, 33, 37, 41]:
                self._walkingdata_cnv(45 - cnv)
#                time.sleep(0.1)
                step = int(args.ws)
  
    def _walkingdata_cnv(self, cnv):

        servo_sn = 0
        target_dm = 10
        yaw_radian = 0.1
        for sn in range(3):
            if cnv > 22: 
	       cnv_r = cnv - 22
	       yaw = -1
            else: 
               cnv_r = cnv + 22
               yaw = 1
            # front left leg
	    if sn in [1]: target_radian = yaw_radian * yaw
            else: target_radian = self.trajs[cnv][target_dm]
            target_radian = target_radian + self.optset[target_dm] + self.joint_gap[servo_sn]
            target_angle = np.clip(int(637 * target_radian + 1500) * 4, 2000, 10000)
            self.pwm.setTarget(servo_sn, target_angle)
 
           # front right leg
	    if sn in [1]: target_radian = (yaw_radian * -1) * yaw
            else: target_radian = self.trajs[cnv_r][target_dm]
            target_radian = ((target_radian + self.optset[target_dm]) * -1) + self.joint_gap[servo_sn + 3]
            target_angle = np.clip(int(637 * target_radian + 1500) * 4, 2000, 10000)
            self.pwm.setTarget(servo_sn + 3, target_angle)

            servo_sn += 1
            target_dm += 1

        servo_sn = 6
        target_dm = 0
        for sn in range(4):
            if cnv > 22: 
	       cnv_r = cnv - 22
	       yaw = -1
            else: 
               cnv_r = cnv + 22
               yaw = 1
            # rear left leg
	    if sn in [1]: target_radian = (yaw_radian * 1) * yaw + 0.02
            else: target_radian = self.trajs[cnv][target_dm]
	    if sn in [2]: target_radian = self.trajs[cnv][target_dm] - 0
            target_radian = target_radian + self.optset[target_dm] + self.joint_gap[servo_sn]
            target_angle = np.clip(int(637 * target_radian + 1500) * 4, 2000, 10000)
            self.pwm.setTarget(servo_sn, target_angle)
            # rear right leg
	    if sn in [1]: target_radian = (yaw_radian * -1) * yaw + 0.02
            else: target_radian = self.trajs[cnv_r][target_dm]
            if sn in [2]: target_radian = self.trajs[cnv_r][target_dm] - 0
            target_radian = ((target_radian + self.optset[target_dm]) * -1) + self.joint_gap[servo_sn + 4]
            target_angle = np.clip(int(637 * target_radian + 1500) * 4, 2000, 10000)
            self.pwm.setTarget(servo_sn + 4, target_angle)

            servo_sn += 1
            target_dm += 1

    def _stand_set(self):

        servo_sn = 0
        target_dm = 10
        for sn in range(3):
            # front left leg
            target_angle = self.stand_set[target_dm] + self.joint_gap[servo_sn] 
            self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle))
            # front right leg
            target_angle = self.stand_set[target_dm + 3] + self.joint_gap[servo_sn + 3]
#            self.pwm.setTarget(servo_sn + 3, self._convert_radian2pulse(target_angle))
            servo_sn += 1
            target_dm += 1

        servo_sn = 6
        target_dm = 0
        for sn in range(4):
            # rear left leg
            target_angle = self.stand_set[target_dm] + self.joint_gap[servo_sn]
            self.pwm.setTarget(servo_sn, self._convert_radian2pulse(target_angle))
            # rear right leg
            target_angle = self.stand_set[target_dm + 4] + self.joint_gap[servo_sn + 4]
#            self.pwm.setTarget(servo_sn + 4, self._convert_radian2pulse(target_angle))

            servo_sn += 1
            target_dm += 1

    def _movement(self, joint_action, joint_gap):

        servo_sn = 0
        set_value_ss = joint_action[3]
        set_value_sa = joint_action[4]
        
        for dt in range(10, 28):
            ss = set_value_ss + joint_action[dt]
            sa = set_value_sa + joint_action[dt + 18]
            target_angle = joint_action[dt + 36] + joint_gap[dt - 10]

#	    if servo_sn not in [6,7,8,9, 14, 15, 16, 17]:
#	    if servo_sn in [14, 15, 16, 17]:
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
        target = np.clip(int(637 * radian_value + 1500) * 4, 2000, 10000)
#        target = np.clip(target, 3000, 9000)
        return target # (pi + 2x) / pi * 1000 + 500

if __name__ == '__main__':
    try:
        mr = MoveACT()
    except KeyboardInterrupt:
        print("Shutting down - MoveACT")
