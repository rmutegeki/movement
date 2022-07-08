#!/usr/bin/env python

import maestro
import time
import subprocess
import numpy as np
import argparse
import sys, os
sys.path.append('./')
from maicat_config import MaicatConfig

parser = argparse.ArgumentParser(description='')
parser.add_argument('-cmd', required=True, help='command num')

args = parser.parse_args()

class MaicatControl:

    def __init__(self):

        f = open("app_position.txt", 'r')
        data = f.readline()
        f.close()

        self.posture = int(data)
        self.cmd_nm_prev = self.posture

	print(self.posture)

        self.pwm = maestro.Controller()
#        time.sleep(0.5)
#        subprocess.call("sudo python cmd_volum.py -v 30", shell=True)

        self.joint_fix = 1
        self.sound_opt = 1

        self.cmd_nm = int(args.cmd)
	self.app_position_save()
        self._command_action()

    def app_position_save(self):

	if self.cmd_nm not in [1,2,3,4,5,6,7,8,9, 21,22,23,24,25,26,27,28, 99, 100]: return

        f = open("app_position.txt", 'w')

	if self.cmd_nm in [99, 100]: data = 7
        else: data = self.cmd_nm

        f.write(str(data))
        f.close()

    def start(self):
	while True:
           self._command_print()

    def _command_print(self):
        os.system('clear')
	
	if self.joint_fix in [0]: joint_fix = "Disabled"
	if self.joint_fix in [1]: joint_fix = "Enabled"

	if self.posture in [1]: posture = "Sitdown"
	if self.posture in [21]: posture = "Standup"
	if self.posture in [7]: posture = "Falldown"

	if self.sound_opt in [1]: sound = "Enabled"
	else: sound = "Disabled"
	print ""
	print "                      M O V E M E N T    C O N T R O L 1.0"
	print ""
	print "   ========================================================================"
	print "   # JOINT FIX:", joint_fix, "  # POSTURE:", posture, "  # SOUND:", sound
	print "   ------------------------------------------------------------------------"
        print "   1. sitdown               21. standup             31. JOINT FIX(on/off)"
        print "   2. sitdown eat           22. shake rl            32. SOUND(on/off)"
        print "   3. sitdown stretch       23. walk Front"
        print "   4. sitdown kneading      24. walk Front Right    41. STATUS: Falldown"
        print "   5. sitdown punch         25. walk Front Left     42. STATUS: Sitdown"
        print "   6. sitdown lick          26. walk Back           43. STATUS: Standup"
        print "   7. sitdown -> falldown   27. walk Back Right"
        print "   8. falldown -> sitdown   28. walk Back Left      51. M/C sitdown motion"
        print "   9. uplift -> ready" 
        print "      ready                                         99. QUIT"
        print "      uplift -> sitdown                             100. shutdown"
	print "   ------------------------------------------------------------------------"
	print "   # HEAD normal : 61. right   62. left   63. center   64. l/r view"
	print "         nodding : 71. right   72. left   73. center   74. shake"
	print "   ------------------------------------------------------------------------"
	print "   # SOUND : 81. normal   82. happy   83. grooming   84. angry"
	print "   ------------------------------------------------------------------------"

        cmd_nm = int(input("   [COMMAND NUM] :  "))
	self.cmd_nm = cmd_nm
	self._command_action()

    def _command_action(self):
	
        if self.cmd_nm in [99]: 
		self.stop()
		exit()
        if self.cmd_nm in [1]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_sitdown2.py -x 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = self.cmd_nm; self.posture = self.cmd_nm_prev
	   if self.cmd_nm_prev in [7]:
	      call_data = "python movement_sitdown2falldown.py -r 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 1; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [2]:
	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_sitdown2eat.py -x 1 -st 2 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.posture = self.cmd_nm_prev

        if self.cmd_nm in [3]:
	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_sitdown2stretch1.py -x 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.posture = self.cmd_nm_prev

        if self.cmd_nm in [4]:
	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_sitdown2kneading.py -x 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.posture = self.cmd_nm_prev

        if self.cmd_nm in [5]:
	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_sitdown2punch.py -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.posture = self.cmd_nm_prev

        if self.cmd_nm in [6]:
	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_sitdown2lick.py -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.posture = self.cmd_nm_prev

        if self.cmd_nm in [7]:
	   if self.cmd_nm_prev in [21]:
	      subprocess.call("python cmd_headaction.py -a 1", shell=True)
	      call_data = "python movement_sitdown2.py -x 1 -c  "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 1; self.posture = self.cmd_nm_prev
	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_sitdown2falldown.py -r 0 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = self.cmd_nm; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [8]:
	   if self.cmd_nm_prev in [7]:
	      call_data = "python movement_sitdown2falldown.py -r 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 1; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [9]:
	   if self.cmd_nm_prev in [7]:
	      call_data = "python movement_uplift.py -r 0 -x 1 -c 1"
	      call_process = call_data
	      subprocess.call(call_process, shell=True)

	      call_data = "python movement_ready.py -c 1"
	      call_process = call_data
	      subprocess.call(call_process, shell=True)

	      call_data = "python movement_uplift.py -r 1 -x 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)

	      self.cmd_nm_prev = 1; self.posture = self.cmd_nm_prev


        if self.cmd_nm in [21]:
	   if self.cmd_nm_prev in [7]:
	      subprocess.call("python cmd_headaction.py -a 1", shell=True)
	      call_data = "python movement_sitdown2falldown.py -r 1 -c  "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 1; self.posture = self.cmd_nm_prev

	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_standup2.py -x 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = self.cmd_nm; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [22]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_shake_rl.py -x 1 -c "
	      call_process = call_data + str(self.joint_fix)

	      if self.cmd_nm_prev in [1]: call_data_n = "-ns -0.5 -ne -0.5 -nm 0.2 -c "
              else: call_data_n = "-ns 0.5 -ne 0.5 -nm 0.5 -c "
	      call_data = " & python movement_head2.py -m 2 "
	      call_process = call_process + call_data + call_data_n + str(self.joint_fix)

	      subprocess.call(call_process, shell=True)
	      self.posture = self.cmd_nm_prev

        if self.cmd_nm in [23]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_walk.py -d Front -so 0.1 -st 3 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 21; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [24]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_walk.py -d FrontRight -so 0.1 -st 2 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 21; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [25]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_walk.py -d FrontLeft -so 0.1 -st 2 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 21; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [26]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_walk.py -d Back -so -0.1 -st 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 21; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [27]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_walk.py -d BackRight -so -0.05 -st 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 21; self.posture = self.cmd_nm_prev

        if self.cmd_nm in [28]:
	   if self.cmd_nm_prev in [21]:
	      call_data = "python movement_walk.py -d BackLeft -so -0.05 -st 1 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 21; self.posture = self.cmd_nm_prev

	if self.cmd_nm in [31]:
	   if self.joint_fix in [1]:
	      self.joint_fix = 0
	   else: self.joint_fix = 1

	if self.cmd_nm in [32]:
	   if self.sound_opt in [1]:
	      self.sound_opt = 0
	      subprocess.call("python cmd_volum.py -v 0", shell=True)
	   else: 
	      self.sound_opt = 1
	      subprocess.call("python cmd_volum.py -v 30", shell=True)

	if self.cmd_nm in [41]: self.posture = 7; self.cmd_nm_prev = self.posture
	if self.cmd_nm in [42]: self.posture = 1; self.cmd_nm_prev = self.posture
	if self.cmd_nm in [43]: self.posture = 21; self.cmd_nm_prev = self.posture


        if self.cmd_nm in [51]:
	   if self.cmd_nm_prev in [21]:
	      subprocess.call("python cmd_headaction.py -a 1", shell=True)
	      call_data = "python movement_sitdown2.py -x 1 -c  "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 1; self.posture = self.cmd_nm_prev

	   if self.cmd_nm_prev in [1]:
	      call_data = "python movement_sitdown2falldown.py -r 0 -c "
	      call_process = call_data + str(self.joint_fix)
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 7; self.posture = self.cmd_nm_prev

	   if self.cmd_nm_prev in [7]:
	      call_data = "python maicat_control.py -status 3"
	      call_process = call_data
	      subprocess.call(call_process, shell=True)
	      self.cmd_nm_prev = 1; self.posture = self.cmd_nm_prev


	if self.cmd_nm_prev in [1]: call_data_n = "-ns -0.5 -ne -0.5 -nm 0.2 -c "
        else: call_data_n = "-ns 0.5 -ne 0.5 -nm 0.5 -c "

	if self.cmd_nm in [61, 62, 63, 64]:
	   call_data = "sudo python movement_head2.py "
  	   if self.cmd_nm in [61]: call_data_m = "-m 7 "
	   if self.cmd_nm in [62]: call_data_m = "-m 8 "
	   if self.cmd_nm in [63]: call_data_m = "-m 6 "
	   if self.cmd_nm in [64]: call_data_m = "-m 2 "
	   call_process = call_data + call_data_m + call_data_n + str(self.joint_fix)
	   subprocess.call(call_process, shell=True)

	if self.cmd_nm in [71, 72, 73, 74]:
	   call_data = "sudo python movement_head2.py "
  	   if self.cmd_nm in [71]: call_data_m = "-m 3 "
	   if self.cmd_nm in [72]: call_data_m = "-m 4 "
	   if self.cmd_nm in [73]: call_data_m = "-m 5 "
	   if self.cmd_nm in [74]: call_data_m = "-m 1 "
	   call_process = call_data + call_data_m + call_data_n + str(self.joint_fix)
	   subprocess.call(call_process, shell=True)

	if self.cmd_nm in [81]: subprocess.call("sudo python cmd_headaction.py -a 1", shell=True)
	if self.cmd_nm in [82]: subprocess.call("sudo python cmd_headaction.py -a 2", shell=True)
	if self.cmd_nm in [83]: subprocess.call("sudo python cmd_headaction.py -a 3", shell=True)
	if self.cmd_nm in [84]: subprocess.call("sudo python cmd_headaction.py -a 4", shell=True)

	if self.cmd_nm in [99]: self.stop(); exit()

        if self.cmd_nm in [100]: 
	   self.stop()
	   call_data = "sudo shutdown now"
	   call_process = call_data
	   subprocess.call(call_process, shell=True)
           exit()

    def stop(self):
	if self.cmd_nm_prev in [21]:
	   call_data = "sudo python movement_sitdown2.py -x 1 -c 1"
	   call_process = call_data
	   subprocess.call(call_process, shell=True)
	   self.cmd_nm_prev = 1;

	subprocess.call("sudo python cmd_headaction.py -a 2", shell=True)

	if self.cmd_nm_prev in [1]:
	   call_data = "sudo python movement_sitdown2falldown.py -r 0 -c 1"
	   call_process = call_data
	   subprocess.call(call_process, shell=True)

        self._disconnect_all()

    def _disconnect_all(self):
        for sn in range(18):
            self.pwm.setTarget(sn, 0)

if __name__ == '__main__':
    try:
        mc = MaicatControl()
#        mc.start()
    except KeyboardInterrupt:
        print("Shutting down - movement_control")
	mc.stop()
