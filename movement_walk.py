#!/usr/bin/env python
'''
Created on 2022. 1. 7.
@author: macroact
'''

import maestro
import time
import math as m
import argparse
import serial

import numpy as np
import sys, os
#import FaBo9Axis_MPU9250

sys.path.append('./')
from data_config import DataConfig
from optset_config import OptsetConfig

parser = argparse.ArgumentParser(description='')
parser.add_argument('-st', required=True, help='Walking step')
parser.add_argument('-so', required=True, help='stride option')
parser.add_argument('-c', required=True, help='servo active 1, close 0')
parser.add_argument('-d', required=True, help='Front, FrontRight, FrontLeft, Back, BackRight, BackLeft')

args = parser.parse_args()

NECK_NOD_SN = 14
HEAD_SHAKE_SN = 15
HEAD_NOD_SN = 16
TAIL_SN = 17

TOF_MAX_RANGE = 4000

class MoveACT:

    def __init__(self):

	f = open("position_data.txt", 'r')
	data = f.readline()
	f.close
#	if data not in ["standup"]: exit()

	self.teensy_io = serial.Serial("/dev/ttyACM0", 9600)
        walking_step = int(args.st)

	self.current_dl = TOF_MAX_RANGE
	self.current_dr = TOF_MAX_RANGE

        self.pwm = maestro.Controller()
        time.sleep(1)

        self.joint_gap = OptsetConfig.JOINT_GAP_0

        joint_action = DataConfig.JOINT_0029
        self._movement(joint_action, self.joint_gap)

	self.walk_type = str(args.d)

	self._joint_data_config()

	self._set_smooth(14, 18, 20, 5)
	self.walk_type_org = self.walk_type
	self.d_limit = 400
	self.t_limit = 500

	if self.walk_type in ["Front"]: self._object_move(0.2, 0, -1, 0)
	if self.walk_type in ["FrontRight"]: self._object_move(0.2, -0.7, -1, 0)
	if self.walk_type in ["FrontLeft"]: self._object_move(0.2, 0.7, -1, 0)
	if self.walk_type in ["Back"]: self._object_move(0.3, -1, -1, 0)
	if self.walk_type in ["BackRight"]: self._object_move(0.3, -1, -1, 0)
	if self.walk_type in ["BackLeft"]: self._object_move(0.3, 1, -1, 0)

	self._set_smooth(0, 14, 100, 0)

	walk_config = np.zeros((2, 4))
	walk_config[0] = [3, 8, 2, 7] # front 4 9 3 8
	walk_config[1] = [3, 8, 2, 7] # back    1 7 9 5

	if self.walk_type in ["Front", "FrontLeft", "FrontRight"]: walk_st_opt = walk_config[0]
 	if self.walk_type in ["Back", "BackLeft", "BackRight"]: walk_st_opt = walk_config[1]

	self.fln = int(walk_st_opt[0])
	self.frn = int(walk_st_opt[1])
	self.rln = int(walk_st_opt[2])
	self.rrn = int(walk_st_opt[3])

        for cn in range(walking_step):
	    for cv in range(2):
#	       self._io_data_read(self.d_limit, self.t_limit)
	       if self.walk_type_org in self.walk_type: self._walking()

#	    self._gyroCheck()

        joint_action = DataConfig.JOINT_0029
        self._movement(joint_action, self.joint_gap)

	if int(args.c) == 1: exit()

	self._servo_disconnect()
        self.pwm.close()

#    def _gyroCheck(self):
#	mpu9250 = FaBo9Axis_MPU9250.MPU9250()
#	pos_Y = 0
#        count_Y = 1
#	for cnt in range(count_Y):
#	   Accel = mpu9250.readAccel()
#	   pos_Y += Accel['y']
#	pos_Y = round((pos_Y / count_Y), 1)
#	if pos_Y > 0.6 or pos_Y < -0.6: 
#	   if int(args.c) == 1: exit()
#	   else:
#	      self._servo_disconnect(sn)
#	      exit() 

    def _io_data_read(self, d_limit, t_limit):
	cnt = 0
	self.current_dl = 0
	self.current_dr = 0
	self._teensy_io_flush()
	while cnt < 2:
	   data = self.teensy_io.readline()
	   if data is not None:
	      outputs = data.split(",")
	      if outputs[0] == "DL": 
                 self.current_dl = int(outputs[1])
	      elif outputs[0] == "DR": 
                 self.current_dr = int(outputs[1])
	   cnt += 1

	if self.walk_type_org in ["Front"]:
	   if self.current_dl > t_limit and self.current_dr > t_limit: self.walk_type = self.walk_type_org
	   if self.current_dl < d_limit and self.current_dr < d_limit: self.walk_type = "Stop"
	   if self.current_dl < t_limit and self.current_dr > t_limit: self.walk_type = "FrontRight"
	   if self.current_dl > t_limit and self.current_dr < t_limit: self.walk_type = "FrontLeft"

#	print self.walk_type_org, self.walk_type, self.current_dr, self.current_dl

    def _walking(self):

	step_cnt = 11
	sn_switch = [3,4,5, 0,1,2, 10,11,12,13, 6,7,8,9]   # r/l reversal

#	time.sleep(0.5)

        for cnt in range(step_cnt):

	    for cnv in range(14):

		sn = self._servo_nm[cnv]

	        if sn in [0,1,2]: target_radian = self.f_trajs[self.fln][sn]
	        if sn in [3,4,5]: target_radian = self.f_trajs[self.frn][sn-3] * -1

	        if sn in [6,7,8,9]: target_radian = self.r_trajs[self.rln][sn-6]
	        if sn in [10,11,12,13]: target_radian = self.r_trajs[self.rrn][sn-10] * -1

		if sn in [1] and self.walk_type in ["Front", "FrontLeft", "FrontRight"]:
		   target_radian = 0.2
		   if cnt > 0 and cnt < 6: target_radian = target_radian + 0.05  # 3, 15
		   else: target_radian = target_radian - 0.05

		if sn in [4] and self.walk_type in ["Front", "FrontLeft", "FrontRight"]:
		   target_radian = -0.2
		   if cnt > 0 and cnt < 6: target_radian = target_radian + 0.05  # 3, 15
		   else: target_radian = target_radian - 0.05

		if self.walk_type in ["FrontLeft", "FrontRight"]:
		   if sn in [0]: target_radian = self._map(target_radian, self.min0, self.max0, self.min0 + 0.2, self.max0 - 0.5)
		   if sn in [6]: target_radian = self._map(target_radian, self.min6, self.max6, self.min6 + 0.15, self.max6 - 0.15)

		if self.walk_type in ["BackLeft", "BackRight"]:
		   if sn in [0]: target_radian = self._map(target_radian, self.min0, self.max0, self.min0 + 0.15, self.max0 - 0.15)
		   if sn in [6]: target_radian = self._map(target_radian, self.min6, self.max6, self.min6 + 0.2, self.max6 - 0.5)

		sn_target = sn
		if self.walk_type in ["FrontRight", "BackRight"]:
		   sn_target = sn_switch[sn_target]
		   target_radian = target_radian * -1


                target_radian = target_radian + self.joint_gap[sn_target]
                target_angle = np.clip(int(637 * target_radian + 1500) * 4, 2000, 10000)

#                if fln not in [18] or frn not in [18]: 
#		if sn_target in [3, 4, 5]:
#		   print self.frn, sn_target, target_radian 
		self.pwm.setTarget(sn_target, target_angle)

	    time.sleep(0.0) # 0.07

	    if self.walk_type in ["Front", "FrontLeft", "FrontRight"]: self.fln += 1; self.frn += 1; self.rln += 1; self.rrn += 1
	    if self.walk_type in ["Back", "BackLeft", "BackRight"]: self.fln -= 1; self.frn -= 1; self.rln -= 1; self.rrn -= 1

	    if self.fln > 11: self.fln = 0
	    if self.frn > 11: self.frn = 0
	    if self.rln > 11: self.rln = 0
	    if self.rrn > 11: self.rrn = 0
	    if self.fln < 0: self.fln = 11
	    if self.frn < 0: self.frn = 11
	    if self.rln < 0: self.rln = 11
	    if self.rrn < 0: self.rrn = 11

    def _object_move(self, neck_y, head_x, head_y, tail_y):
	neck_y = neck_y + self.joint_gap[NECK_NOD_SN]
	head_x = head_x + self.joint_gap[HEAD_SHAKE_SN]
	head_y = head_y + self.joint_gap[HEAD_NOD_SN]
	tail_y = tail_y + self.joint_gap[TAIL_SN]

	self.pwm.setTarget(NECK_NOD_SN, self._convert_radian2pulse(neck_y))
	self.pwm.setTarget(HEAD_SHAKE_SN, self._convert_radian2pulse(head_x))
	self.pwm.setTarget(HEAD_NOD_SN, self._convert_radian2pulse(head_y))
	self.pwm.setTarget(TAIL_SN, self._convert_radian2pulse(tail_y))
	
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

    def _map(self, x, input_min, input_max, output_min, output_max):
	return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min

    def _set_smooth(self, sn_st, sn_ed, ss, sa):
	for sn in range(sn_st, sn_ed):
	    self.pwm.setSpeed(sn,ss)
	    self.pwm.setAccel(sn,sa)

    def _set_leg_accel(self, sa):
	for sn in range(14):
	    self.pwm.setAccel(sn,sa)

    def _servo_disconnect(self):
        for sn in range(18):
            self._disconnect(sn)

    def _disconnect(self, sn):
        self.pwm.setTarget(sn, 0)

    def _convert_radian2pulse(self, radian_value):
        target = np.clip(int(637 * radian_value + 1500) * 4, 2000, 10000)
        return target # (pi + 2x) / pi * 1000 + 500

    def _teensy_io_flush(self):
	self.teensy_io.flushInput()
	self.teensy_io.flushOutput()

    def _joint_data_config(self):

	self._trajs_set()

	stride_option = float(args.so)
	stride_lim = stride_option

	for cnt in range(12):
	    tra = float(self.f_trajs[cnt][0])
	    if cnt == 0: max0 = tra; min0 = tra
	    if max0 < tra: max0 = tra
	    if min0 > tra: min0 = tra

	    tra = float(self.f_trajs[cnt][2])
	    if cnt == 0: max2 = tra; min2 = tra
	    if max2 < tra: max2 = tra
	    if min2 > tra: min2 = tra

	if self.walk_type in ["Front", "FrontLeft", "FrontRight"]: walk_dir_optset = 0.1
	if self.walk_type in ["Back", "BackLeft", "BackRight"]: walk_dir_optset = 0.1   # -0.2
	
	for cnt in range(12):
	    tra = float(self.f_trajs[cnt][0])
	    self.f_trajs[cnt][0] = self._map(tra, min0, max0, min0 + walk_dir_optset + stride_lim, max0 - stride_lim)

	    tra = float(self.f_trajs[cnt][2])
	    if self.walk_type not in ["Front", "Back"]:
  	       self.f_trajs[cnt][2] = self._map(tra, min2, max2, min2 - 0.5, max2 + 0.3)   # min - 0.1     max2 + 0.3
	    else: self.f_trajs[cnt][2] = self._map(tra, min2, max2, min2 - 0.2, max2 + 0.5)   # min - 0.1     max2 + 0.3

	self.max0 = max0
	self.min0 = min0
	self.max2 = max2
	self.min2 = min2

	for cnt in range(12):
	    tra = float(self.r_trajs[cnt][0])
	    if cnt == 0: max6 = tra; min6 = tra
	    if max6 < tra: max6 = tra
	    if min6 > tra: min6 = tra

	    tra = float(self.r_trajs[cnt][2])
	    if cnt == 0: max8 = tra; min8 = tra
	    if max8 < tra: max8 = tra
	    if min8 > tra: min8 = tra

	    tra = float(self.r_trajs[cnt][3])
	    if cnt == 0: max9 = tra; min9 = tra
	    if max9 < tra: max9 = tra
	    if min9 > tra: min9 = tra

	for cnt in range(12):
	    tra = float(self.r_trajs[cnt][0])
	    self.r_trajs[cnt][0] = self._map(tra, min6, max6, min6 - 0.1 + stride_lim, max6 - stride_lim + 0.1)

	    tra = float(self.r_trajs[cnt][2])
	    self.r_trajs[cnt][2] = self._map(tra, min8, max8, min8 - 0.4, max8)

	    tra = float(self.r_trajs[cnt][3])
	    self.r_trajs[cnt][3] = self._map(tra, min9, max9, min9 - 0.5, max9 + 0.5)

	self.max6 = max6
	self.min6 = min6
	self.max8 = max8
	self.min8 = min8

	self._servo_nm = [0, 3, 6, 10, 1, 4, 7, 11, 2, 5, 8, 12, 9, 13]

    def _trajs_set(self):
        # front left leg trajectory
        self.f_trajs = np.zeros((12, 3))
        self.f_trajs[0] = [-0.55,0.0,-1.57]
        self.f_trajs[1] = [-0.34,0.0,-1.27]
        self.f_trajs[2] = [-0.19,0.0,-1.08]
        self.f_trajs[3] = [-0.06,0.0,-0.98]
        self.f_trajs[4] = [0.04,0.0,-0.94]
        self.f_trajs[5] = [0.12,0.0,-0.94]
        self.f_trajs[6] = [0.19,0.0,-0.98]
        self.f_trajs[7] = [0.3,0.0,-0.82]
        self.f_trajs[8] = [0.56,0.0,-0.65]   #-0.27 
        self.f_trajs[9] = [0.35,0.0,-0.65]   #-0.27
        self.f_trajs[10] = [-0.06,0.0,-0.68] #-0.53
        self.f_trajs[11] = [-0.55,0.0,-1.57]
	motor_direction = [0.2, 0.1, -0.2] #0.1, 0, -0.2
        self.f_trajs = np.add(self.f_trajs, motor_direction)

        # rear left leg trajectory
        self.r_trajs = np.zeros((12, 4))
        self.r_trajs[0] = [0.07,0.0,0.51,-0.71] # 0.51 -0.71
        self.r_trajs[1] = [0.21,0.0,0.52,-0.73] # 0.52 -0.73
        self.r_trajs[2] = [0.18,0.0,0.21,-0.69]  # 0.21 -0.69
        self.r_trajs[3] = [-0.19,0.0,0.06,-0.69] #0.06 -0.69
        self.r_trajs[4] = [-0.56,0.0,0.11,-0.69] #0.11 -0.69
        self.r_trajs[5] = [-0.60,0.0,0.58,-0.81] #0.58 -0.81
        self.r_trajs[6] = [-0.37,0.0,0.75,-1.04]  #0.75 -1.04
        self.r_trajs[7] = [-0.33,0.0,0.64,-0.84]  #0.64 -0.84
        self.r_trajs[8] = [-0.27,0.0,0.56,-0.78] # 0.56 -0.78
        self.r_trajs[9] = [-0.19,0.0,0.51,-0.71] # 0.51 -0.71
        self.r_trajs[10] = [-0.09,0.0,0.49,-0.69] # 0.49 -0.69
        self.r_trajs[11] = [-0.00,0.0,0.49,-0.69] # 0.49 -0.69
	motor_direction = [-0.1, 0, -0.2, 0.3]
        self.r_trajs = np.add(self.r_trajs, motor_direction)

if __name__ == '__main__':
    try:
        mr = MoveACT()
    except KeyboardInterrupt:
#	self._servo_disconnect()
        self.pwm.close()
        print("Shutting down - MoveACT")
