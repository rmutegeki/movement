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
parser.add_argument('-c', required=True, help='servo active 1, close 0')
parser.add_argument('-d', required=True, help='direction/  straight 0, right n, left -n ')

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

        self.joint_gap = DataConfig.JOINT_GAP_0

        joint_action = DataConfig.JOINT_0029
        self._movement(joint_action, self.joint_gap)

	self._joint_data_config()

	self._set_smooth(0, 0)

	stn = 3
	fln = stn
	frn = stn + 12  #13  13
	rln = stn + 20  #20  18
	rrn = stn + 11  #11 7

	if frn > 22: frn = frn - 23
	if rln > 22: rln = rln - 23
	if rrn > 22: rrn = rrn - 23

#	print fln, frn, rln, rrn

        for cn in range(walking_step):

            self._walking(2, fln, frn, rln, rrn)
#	    self._gyroCheck()

        time.sleep(0.02)

        joint_action = DataConfig.JOINT_0029
        self._movement(joint_action, self.joint_gap)

	if int(args.c) == 1: exit()

	self._servo_disconnect()
        self.pwm.close()

#    def _gyroCheck(self):
#	mpu9250 = FaBo9Axis_MPU9250.MPU9250()
#	pos_Y = 0
 #       count_Y = 1
#	for cnt in range(count_Y):
#	   Accel = mpu9250.readAccel()
#	   pos_Y += Accel['y']
#	pos_Y = round((pos_Y / count_Y), 1)
#	if pos_Y > 0.6 or pos_Y < -0.6: 
#	   if int(args.c) == 1: exit()
#	   else:
#	      self._servo_disconnect(sn)
#	      exit() 

    def _walking(self, ssp, fln, frn, rln, rrn):

        for cnt in range(23):

	    for cnv in range(14):
		sn = self._servo_nm[cnv]
	        if sn in [0,1,2]: target_radian = self.f_trajs[fln][sn]
	        if sn in [3,4,5]: target_radian = self.f_trajs[frn][sn-3] * -1

	        if sn in [6,7,8,9]: target_radian = self.r_trajs[rln][sn-6]
	        if sn in [10,11,12,13]: target_radian = self.r_trajs[rrn][sn-10] * -1
		
                target_radian = target_radian + self.joint_gap[sn]
                target_angle = np.clip(int(637 * target_radian + 1500) * 4, 2000, 10000)
                self.pwm.setTarget(sn, target_angle)

	    fln += ssp; frn += ssp; rln += ssp; rrn += ssp
	    if fln > 22: fln = 0
	    if frn > 22: frn = 0
	    if rln > 22: rln = 0
	    if rrn > 22: rrn = 0
	    time.sleep(0.01)
#	    print cnt
#	    a=input()


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

    def _map(self, x, input_min, input_max, output_min, output_max):
	return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min

    def _set_smooth(self, ss, sa):
	target_angle = 0
	for sn in range(18):
	    self.pwm.setSpeed(sn,ss)
	    self.pwm.setAccel(sn,sa)

    def _servo_disconnect(self):
        for sn in range(18):
            self._disconnect(sn)

    def _disconnect(self, sn):
        self.pwm.setTarget(sn, 0)

    def _convert_radian2pulse(self, radian_value):
        target = np.clip(int(637 * radian_value + 1500) * 4, 2000, 10000)
        return target # (pi + 2x) / pi * 1000 + 500


    def _joint_data_config(self):

        # front left leg trajectory
        self.f_trajs = np.zeros((23, 3))
        self.f_trajs[0] = [-0.55,0.10,-1.57]
        self.f_trajs[1] = [-0.52,0.10,-1.57]
        self.f_trajs[2] = [-0.34,0.10,-1.27]
        self.f_trajs[3] = [-0.25,0.10,-1.16]
        self.f_trajs[4] = [-0.19,0.10,-1.08]
        self.f_trajs[5] = [-0.12,0.10,-1.02]
        self.f_trajs[6] = [-0.06,0.10,-0.98]
        self.f_trajs[7] = [-0.01,0.10,-0.94]
        self.f_trajs[8] = [0.04,0.10,-0.94]
        self.f_trajs[9] = [0.08,0.10,-0.94]
        self.f_trajs[10] = [0.12,0.10,-0.94]
        self.f_trajs[11] = [0.16,0.10,-0.95]
        self.f_trajs[12] = [0.19,0.10,-0.98]
        self.f_trajs[13] = [0.21,0.10,-0.92]
        self.f_trajs[14] = [0.30,0.10,-0.82]
        self.f_trajs[15] = [0.45,0.10,-0.38]
        self.f_trajs[16] = [0.56,0.10,-0.27]
        self.f_trajs[17] = [0.49,0.10,-0.23]
        self.f_trajs[18] = [0.35,0.10,-0.27]
        self.f_trajs[19] = [0.16,0.10,-0.43]
        self.f_trajs[20] = [-0.06,0.10,-0.53]
        self.f_trajs[21] = [-0.25,0.10,-0.73]
        self.f_trajs[22] = [-0.55,0.10,-1.57]
	motor_direction = [0.3, 0, 0]
        self.f_trajs = np.add(self.f_trajs, motor_direction)

	for cnt in range(23):
	    tra = float(self.f_trajs[cnt][0])
	    if cnt == 0: max0 = tra; min0 = tra
	    if max0 < tra: max0 = tra
	    if min0 > tra: min0 = tra

	    tra = float(self.f_trajs[cnt][2])
	    if cnt == 0: max2 = tra; min2 = tra
	    if max2 < tra: max2 = tra
	    if min2 > tra: min2 = tra

	for cnt in range(23):
	    tra = float(self.f_trajs[cnt][0])
	    self.f_trajs[cnt][0] = self._map(tra, min0, max0, min0 - 0.2, max0)

	    tra = float(self.f_trajs[cnt][2])
	    self.f_trajs[cnt][2] = self._map(tra, min2, max2, min2, max2 + 0.2)

        # rear left leg trajectory
        self.r_trajs = np.zeros((23, 4))
        self.r_trajs[0] = [0.07,0.10,0.51,-0.71]
        self.r_trajs[1] = [0.13,0.10,0.52,-0.74]
        self.r_trajs[2] = [0.21,0.10,0.52,-0.73]
        self.r_trajs[3] = [0.23,0.10,0.36,-0.69]
        self.r_trajs[4] = [0.18,0.10,0.21,-0.69]
        self.r_trajs[5] = [-0.02,0.10,0.08,-0.69]
        self.r_trajs[6] = [-0.19,0.10,0.06,-0.69]
        self.r_trajs[7] = [-0.42,0.10,0.06,-0.69]
        self.r_trajs[8] = [-0.56,0.10,0.11,-0.69]
        self.r_trajs[9] = [-0.66,0.10,0.31,-0.69]
        self.r_trajs[10] = [-0.60,0.10,0.58,-0.81]
        self.r_trajs[11] = [-0.44,0.10,0.79,-1.10]
        self.r_trajs[12] = [-0.37,0.10,0.75,-1.04]
        self.r_trajs[13] = [-0.35,0.10,0.69,-0.96]
        self.r_trajs[14] = [-0.33,0.10,0.64,-0.84]
        self.r_trajs[15] = [-0.30,0.10,0.59,-0.83]
        self.r_trajs[16] = [-0.27,0.10,0.56,-0.78]
        self.r_trajs[17] = [-0.23,0.10,0.53,-0.74]
        self.r_trajs[18] = [-0.19,0.10,0.51,-0.71]
        self.r_trajs[19] = [-0.14,0.10,0.49,-0.69]
        self.r_trajs[20] = [-0.09,0.10,0.49,-0.69]
        self.r_trajs[21] = [-0.04,0.10,0.49,-0.69]
        self.r_trajs[22] = [-0.00,0.10,0.49,-0.69]
	motor_direction = [-0.1, -0.05, -0.4, 0.3]
        self.r_trajs = np.add(self.r_trajs, motor_direction)

	for cnt in range(23):
	    tra = float(self.r_trajs[cnt][0])
	    if cnt == 0: max0 = tra; min0 = tra
	    if max0 < tra: max0 = tra
	    if min0 > tra: min0 = tra

	    tra = float(self.r_trajs[cnt][2])
	    if cnt == 0: max2 = tra; min2 = tra
	    if max2 < tra: max2 = tra
	    if min2 > tra: min2 = tra

	for cnt in range(23):
	    tra = float(self.f_trajs[cnt][0])
	    self.f_trajs[cnt][0] = self._map(tra, min0, max0, min0, max0)

	    tra = float(self.r_trajs[cnt][2])
	    self.r_trajs[cnt][2] = self._map(tra, min2, max2, min2 - 0.4, max2)

	self._servo_nm = [0, 3, 6, 10, 1, 4, 7, 11, 2, 5, 8, 12, 9, 13]

if __name__ == '__main__':
    try:
        mr = MoveACT()
    except KeyboardInterrupt:
	self._servo_disconnect()
        self.pwm.close()
        print("Shutting down - MoveACT")
