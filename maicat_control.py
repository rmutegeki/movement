import numpy as np
import maestro
#from threading import Thread
import serial
import random
import subprocess
import argparse
import time
import sys
sys.path.append('./')
from data_config import DataConfig
from optset_config import OptsetConfig

parser = argparse.ArgumentParser(description='')
parser.add_argument('-status', required=False, default=3, help='status')
args = parser.parse_args()

JOINTS_CNT = 18
SMOOTH_SPEED = 0
SMOOTH_ACCEL = 5

HEAD_SN_MIN = 14
HEAD_SHAKE_SN = 15
HEAD_NOD_SN = 16
TAIL_SN = 17

HEAD_SHAKE_CENTER_RAD = 0.0
HEAD_NOD_CENTER_RAD = -1.0
TAIL_CENTER_RAD = 0.5

HEAD_NOD_LIMIT = [-1.3, -0.6]
TAIL_LIMIT = [-0.5, 1]

DISTANCE_QUEUE_SIZE = 5
TOF_MAX_RANGE = 4000

class MaicatControl:

    def __init__(self):
#        time.sleep(5)
        self.pwm = maestro.Controller()
        time.sleep(0.5)
        self._set_smooth()
        
        self.teensy_io = serial.Serial("/dev/ttyACM0", 9600)
        self.teensy_io.flush()
#        self.teensy_io.write('v,30')  # mp3 volum (0~30)
        self.teensy_io.write('ss,0')  # sound sensor disabled
        self.teensy_io.write('nt,3.5') # sound senser target noise
        self.current_dl = TOF_MAX_RANGE
        self.current_dr = TOF_MAX_RANGE
        self.last_dl = TOF_MAX_RANGE
        self.last_dr = TOF_MAX_RANGE
        self.next_time_to_action = self._time_to_perform_action()
        
        self.head_x = HEAD_SHAKE_CENTER_RAD
        self.head_y = HEAD_NOD_CENTER_RAD
        
        self.joint_gap = OptsetConfig.JOINT_GAP_0
        self._is_moving = False
        self._is_center = False
	self._status = int(args.status)
	if self._status in [9]: self._status = random.randint(0, 2)

        # Move to center
        self._move_to_center()


    def start(self):
        # start the thread to read frames from the video stream
	self._pose_ready()
	self._update()
#	self._walking()

#         Thread(target=self._update, args=()).start()
#         return self

    def _time_to_perform_action(self):
        delay_minutes = random.randint(10, 30) # 1-3 minutes 3m = 180s   10~30
        return time.time() + delay_minutes

    def _update(self):
	flushInput_reset =1000
 	reset_cnt = 0
        self.pwm.setAccel(15, 5)
        self.pwm.setAccel(16, 5)
        self.pwm.setAccel(17, 5)
        while True:
            data = self.teensy_io.readline()

	    if reset_cnt > flushInput_reset: 
		self._teensy_io_flush()
		reset_cnt = 0
	    else: reset_cnt += 1

##            print(reset_cnt, data)

            if data is not None and not self._is_moving:
                outputs = data.split(",")
                # Distance
                if outputs[0] == "DL":
                    self.current_dl = int(outputs[1])
                    self._detect_obj()
                elif outputs[0] == "DR":
                    self.current_dr = int(outputs[1])
                    self._detect_obj()

                # Noise
                elif outputs[0] == "NL" and self._is_center: self._nod_left(); time.sleep(1)
                elif outputs[0] == "NR" and self._is_center: self._nod_right(); time.sleep(1)
                elif outputs[0] == "NC" and self._is_center: self._nod_center(); time.sleep(1)

                # Touch
                if outputs[0][0:2] == "T1":
                    self.teensy_io.write('a,2')
                    time.sleep(2)
                elif outputs[0][0:2] == "T2":
                    self.teensy_io.write('a,4')
                    if self._status in [1]: 
			if random.randint(0, 2) in [0]:
			   subprocess.call("sudo python movement_sitdown2.py -c 1 -x 1", shell=True)
			   self._disconnect_range(0, 14)
			   self._status = 0
                elif outputs[0][0:2] == "T3":
                    self.teensy_io.write('a,1')
                    if self._status in [1]:
			if random.randint(0, 2) in [0]:
			   subprocess.call("sudo python movement_sitdown2.py -c 1 -x 1", shell=True)
			   self._disconnect_range(0, 14)
			   self._status = 0

    def _detect_obj(self):
        if 200 > self.current_dl and 200 > self.current_dr:
            self.teensy_io.write('a,4')
            if self._status in [0]:
		subprocess.call("sudo python movement_standup2.py -c 1 -x 1", shell=True)
	        self._status = 1
	    else: time.sleep(2)
	elif 800 < self.current_dl and 800 < self.current_dr: self._move_to_center()
        else: self._move_head()

        if time.time() > self.next_time_to_action: self._random_action()

    def _random_action(self):
	action_nm = random.randint(1, 7) # 1-6
	if self._status in [0]: # when sitting
	    if action_nm in [1]: self._lick()
	    if action_nm in [2]: self._high_five()
	    if action_nm in [3]:
		self.teensy_io.write('a,1')
		subprocess.call("sudo python movement_sitdown2eat.py -c 1 -x 1 -st 2", shell=True)
		self._disconnect_range(0, 14)
	    if action_nm in [4]:
		self.teensy_io.write('a,1')
		subprocess.call("sudo python movement_sitdown2stretch1.py -c 1 -x 1", shell=True)
		self._disconnect_range(0, 14)
	    if action_nm in [5]:
		self.teensy_io.write('a,1')
		subprocess.call("sudo python movement_sitdown2kneading.py -c 1 -x 1", shell=True)
		self._disconnect_range(0, 14)
	    if action_nm in [6]:
		self.teensy_io.write('a,1')
		subprocess.call("sudo python movement_standup2.py -c 1 -x 1", shell=True)
	        self._status = 1
	    if action_nm in [7]:
		self.teensy_io.write('a,1')
        	subprocess.call("sudo python movement_sitdown2falldown.py -c 1 -r 0", shell=True)
		self._disconnect_range(0, 14)
		self._status = 2
        elif self._status in [1]:
	    if action_nm in [1, 2, 3]: self._walking(); action_nm = random.randint(1, 7)
	    if action_nm in [4, 5, 6]: self._shake()
	    if action_nm in [4, 5, 6, 7]:
		self.teensy_io.write('a,3')
		subprocess.call("sudo python movement_sitdown2.py -c 1 -x 1", shell=True)
		self._disconnect_range(0, 14)
		self._status = 0
	elif self._status in [2, 3]:
	    if action_nm in [1, 2, 3] and self._status in [2]:
		self.teensy_io.write('a,1')
		subprocess.call("sudo python movement_sitdown2falldown.py -c 1 -r 1", shell=True)
		self._disconnect_range(0, 14)
		self._status = 0
	    self._nod_head_random()
	    time.sleep(0.5)

	self._teensy_io_flush()
	self._set_smooth_head()
	self.next_time_to_action = self._time_to_perform_action()

    def _move_head(self):

	if not self._is_moving:
            # Move head
	    move_st = 0.1

            if self.current_dl + 300 < self.current_dr: self.head_x += move_st
            elif self.current_dl > self.current_dr + 300: self.head_x -= move_st

            if self.last_dl + 1200 < self.current_dl or self.last_dr + 1200 < self.current_dr: self.head_y += move_st
            else: self.head_y -= move_st

            self.head_x = np.clip(self.head_x, -1.42, 1.42)
            self.head_y = np.clip(self.head_y, HEAD_NOD_LIMIT[0], HEAD_NOD_LIMIT[1])
            tail_y = np.clip((self.head_x / 4), TAIL_LIMIT[0], TAIL_LIMIT[1])

            self.pwm.setTarget(HEAD_SHAKE_SN, np.clip(int(637 * self.head_x + 1500) * 4, 3000, 9000))
            self.pwm.setTarget(HEAD_NOD_SN, np.clip(int(637 * self.head_y + 1500) * 4, 3000, 9000))
            self.pwm.setTarget(TAIL_SN, np.clip(int(637 * tail_y + 1500) * 4, 3000, 9000))

            self.last_dl = self.current_dl
            self.last_dr = self.current_dr

	    neck_lift = -0.2
	    if self._status in [1, 2]: neck_lift = neck_lift + 0.8  # _status 0.sitdown 1.standup

	    self.pwm.setTarget(14, self._convert_radian2pulse(neck_lift))

        if self._is_center:
	    self._teensy_io_flush()
            self._is_center = False
            self.teensy_io.write('ss,0')

    def _walking(self):
	self._status = 1
	self._shake()
	d_limit = 600
	df_limit = 400
	dfloor_limit = 600
	cnt_limit = 6
	call_data = "sudo python movement_walk.py -c 1 -d "
	action = "Front -so 0 -st " + str(random.randint(1, 2))

	joint_action = [ 20, 20, 0.8, -0.5, 0, -0.5 ]
	self._movement_head(joint_action)
	time.sleep(0.5)
	self._io_data_read(cnt_limit)
	current_floor = (self.current_dl + self.current_dr) / 2

	joint_action = [ 100, 20, 0.8, 0.1, 0.8, -1 ]
	self._movement_head(joint_action)
	time.sleep(0.5)
	self._io_data_read(cnt_limit)
	current_dl = (self.current_dl + self.current_dr) / 2

	joint_action = [ 100, 20, 1.2, 0.1, -0.8, -1 ]
	self._movement_head(joint_action)
	time.sleep(0.5)
	self._io_data_read(cnt_limit)
	current_dr = (self.current_dl + self.current_dr) / 2

	joint_action = [ 20, 5, 0.8, 0.1, 0, -1 ]
	time.sleep(0.5)
	self._movement_head(joint_action)
	self._io_data_read(cnt_limit)
	current_d_front = (self.current_dl + self.current_dr) / 2

#	print "front:", current_d_front," floor:", current_floor, " left:", current_dl, "  right:", current_dr

	if current_dl < d_limit: action = "FrontRight -so 0.1 -st 2"
	elif current_dr < d_limit: action = "FrontLeft -so 0.1 -st 2"

	if current_floor > dfloor_limit or self.current_dl < df_limit or self.current_dr < df_limit: action = "Back -so 0.1 -st 2"

	call_process = call_data + action
        subprocess.call(call_process, shell=True)

    def _io_data_read(self, cnt_limit):
	cnt = 0
	dl_chk_cnt = 0
	dr_chk_cnt = 0
	self.current_dl = 0
	self.current_dr = 0
	self._teensy_io_flush()
	while cnt < cnt_limit:
	   data = self.teensy_io.readline()
	   if data is not None:
	      outputs = data.split(",")
	      if outputs[0] == "DL": 
		 self.current_dl = self.current_dl + int(outputs[1])
	         dl_chk_cnt += 1
	      elif outputs[0] == "DR": 
		 self.current_dr = self.current_dr + int(outputs[1])
		 dr_chk_cnt += 1
	   cnt += 1
	self.current_dl = self.current_dl / dl_chk_cnt
	self.current_dr = self.current_dr / dr_chk_cnt

    def _nod_center(self):
	self.teensy_io.write('a,1')
        if not self._is_moving:
	    if self._status in [1, 2]: subprocess.call("sudo python movement_head2.py -m 5 -c 1 -posture 1", shell=True)
	    else: subprocess.call("sudo python movement_head2.py -m 5 -c 1 -posture 0", shell=True)
	self._teensy_io_flush()

    def _nod_right(self):
	self.teensy_io.write('a,1')
        if not self._is_moving:
	    if self._status in [1, 2]: subprocess.call("sudo python movement_head2.py -m 3 -c 1 -posture 1", shell=True)
	    else: subprocess.call("sudo python movement_head2.py -m 3 -c 1 -posture 0", shell=True)
	self._teensy_io_flush()

    def _nod_left(self):
	self.teensy_io.write('a,1')
        if not self._is_moving:
	    if self._status in [1, 2]: subprocess.call("sudo python movement_head2.py -m 4 -c 1 -posture 1", shell=True)
	    else: subprocess.call("sudo python movement_head2.py -m 4 -c 1 -posture 0", shell=True)
	self._teensy_io_flush()

    def _nod_lr(self):
	self.teensy_io.write('a,2')
        if not self._is_moving:
	    if self._status in [1, 2]: subprocess.call("sudo python movement_head2.py -m 2 -c 1 -posture 1", shell=True)
	    else: subprocess.call("sudo python movement_head2.py -m 2 -c 1 -posture 0", shell=True)
	self._teensy_io_flush()

    def _nod_head_random(self):
	self.teensy_io.write('a,2')
        if not self._is_moving:
	    if self._status in [1, 2]: subprocess.call("sudo python movement_head2.py -posture 1 -c 1", shell=True)
	    else: subprocess.call("sudo python movement_head2.py -posture 0 -c 1", shell=True)
	self._teensy_io_flush()

    def _set_start_head(self):
        joint_action = [ 20, 5, 1, -0.5, 0, -1 ]
        self._movement_head(joint_action)

    def _set_end_head(self):
        joint_action = [ 20, 5, 1, -0.7, 0, -1 ]
        self._movement_head(joint_action)

    def _set_smooth(self):
        for sn in range(JOINTS_CNT):
            self.pwm.setSpeed(sn,SMOOTH_SPEED)
            self.pwm.setAccel(sn,SMOOTH_ACCEL)

    def _set_smooth_head(self):
        for sn in range(14, 17):
            self.pwm.setSpeed(sn,SMOOTH_SPEED)
            self.pwm.setAccel(sn,SMOOTH_ACCEL)

    def _convert_radian2pulse(self, radian_value):
        target = int(637 * radian_value + 1500) * 4
        target = np.clip(target, 2000, 10000)
        return target

    def _move_to_center(self):
        self.pwm.setTarget(HEAD_SHAKE_SN, self._convert_radian2pulse(HEAD_SHAKE_CENTER_RAD))
        self.pwm.setTarget(HEAD_NOD_SN, self._convert_radian2pulse(HEAD_NOD_CENTER_RAD))
        self.pwm.setTarget(TAIL_SN, self._convert_radian2pulse(TAIL_CENTER_RAD))
        self.head_x = HEAD_SHAKE_CENTER_RAD
        self.head_y = HEAD_NOD_CENTER_RAD
        if not self._is_center:
	    self._teensy_io_flush()
	    self._is_center = True
	    self.teensy_io.write('ss,1')

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

    def _movement_head(self, joint_action):
        set_value_ss = joint_action[0]
        set_value_sa = joint_action[1]
        head_sn = 3
        for sn in range(14, 17):
            ss = set_value_ss
            sa = set_value_sa
            target_angle = joint_action[head_sn]

	    if sn == 14 and self._status in [1, 2]: target_angle = target_angle + 0.7  # _status  0.sitdown   1.standup   2.uplift

            self.pwm.setSpeed(sn, ss)
            self.pwm.setAccel(sn, sa)
            self.pwm.setTarget(sn, self._convert_radian2pulse(target_angle))
            head_sn += 1

        ts = joint_action[2]
        time.sleep(ts)

    def _high_five(self):
        if not self._is_moving:
            subprocess.call("sudo python movement_sitdown2punch.py -c 1", shell=True)
            self._disconnect_range(6, 14)
            self._is_moving = False

    def _lick(self):
        if not self._is_moving:
            subprocess.call("sudo python movement_sitdown2lick.py -c 1", shell=True)
            self._disconnect_range(6, 14)
            self._is_moving = False
    def _shake(self):
        if not self._is_moving:
            subprocess.call("sudo python movement_shake_rl.py -c 1 -x 1", shell=True)
            self._is_moving = False

    def _teensy_io_flush(self):
        self.teensy_io.flushInput()
        self.teensy_io.flushOutput()

    def _pose_ready(self):
#
# status 
# 	0,ready->sitdown	1,falldown->standup	2,falldown
#
	if self._status in [0]:
	   subprocess.call("sudo python movement_uplift.py -c 1 -x 1 -r 0", shell=True)
           subprocess.call("sudo python movement_ready.py -c 1", shell=True)
           subprocess.call("sudo python movement_uplift.py -c 1 -x 1 -r 1", shell=True)
           subprocess.call("sudo python movement_sitdown2stretch1.py -c 1 -x 1", shell=True)
           joint_action = DataConfig.JOINT_0036
           self._movement(joint_action, self.joint_gap)
           self._disconnect_range(6, 14)

	if self._status in [1]:
	   subprocess.call("sudo python movement_sitdown2falldown.py -c 1 -r 1", shell=True)
	   subprocess.call("sudo python movement_standup2.py -c 1 -x 1", shell=True)

	if self._status in [2]:
           joint_action = DataConfig.JOINT_0237
           self._movement(joint_action, self.joint_gap)
           self._disconnect_range(0, 14)

	if self._status in [3]:
           subprocess.call("sudo python movement_sitdown2falldown.py -c 1 -r 1", shell=True)
           self._disconnect_range(6, 14)
#	   self._status = 0

    def _disconnect(self, sn):
        self.pwm.setTarget(sn, 0)
	self._teensy_io_flush()

    def _disconnect_all(self):
        for sn in range(JOINTS_CNT):
            self.pwm.setTarget(sn, 0)
	self._teensy_io_flush()

    def _disconnect_range(self, start, end):
        for sn in range(start, end):
            self.pwm.setTarget(sn, 0)

	self._set_smooth_head()
	self._teensy_io_flush()

#        self.pwm.setTarget(14, 0) # neck

    def stop(self):
        self._disconnect_all()


if __name__ == '__main__':
    try:
        mc = MaicatControl()
        mc.start()
    except KeyboardInterrupt:
        print("Shutting down - MaicatControl")
        mc.stop()
