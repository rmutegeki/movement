#!/usr/bin/env python
'''
Created on 2020. 11. 19.
@author: macroact
'''

import maestro
import time
import numpy as np
import sys, os
sys.path.append('./')
from maicat_config import MaicatConfig


class MaicatReady1:

    def __init__(self):
        self.pwm = maestro.Controller()
        time.sleep(1)

        # Ready
        #self._set_motor_limits()
        self._set_smooth()

        #self._ready()

        test_type = int(input('TEST Type = '))
        if test_type == 1 :
            self._servochk1()
        else :
            self._servochk2()

        self.pwm.close()

    def _servochk1(self):
        servo_sn = int(input('3/1. Sevo SN = '))
        start_position = float(input('3/2. Start position = '))
        end_position = float(input('3/3. End position = '))

        self.pwm.setSpeed(servo_sn, 100)
        self.pwm.setAccel(servo_sn, 5)

	self.pwm.stopScript()

        self.pwm.setTarget(servo_sn, self._convert_radian2pulse(start_position))
        time.sleep(2)

        self.pwm.setTarget(servo_sn, self._convert_radian2pulse(end_position))
        time.sleep(2)

        self._disconnect(servo_sn)

    def _servochk2(self):
        servo_sn = int(input('Sevo SN = '))

        for sn in range(18):
            joint_name = MaicatConfig.JOINT_NAMES[sn]
            zero = MaicatConfig.JOINT_READY[joint_name]
            if sn == servo_sn :
                self.pwm.setTarget(sn, self._convert_radian2pulse(zero))
        time.sleep(1)

        self._disconnect(servo_sn)

    def _ready(self):
        for sn in range(18):
            joint_name = MaicatConfig.JOINT_NAMES[sn]
            zero = MaicatConfig.JOINT_READY[joint_name]
            self.pwm.setTarget(sn, self._convert_radian2pulse(zero))
        time.sleep(2)

        for sn in range(18):
            self._disconnect(sn)

    def _set_smooth(self):
        for sn in range(18):
            self.pwm.setSpeed(sn, 20)
            self.pwm.setAccel(sn, 5)

    def _set_dynamic(self):
        for sn in range(18):
            self.pwm.setSpeed(sn, 0)
            self.pwm.setAccel(sn, 0)

    def _set_motor_limits(self):
        for sn in range(18):
            joint_name = MaicatConfig.JOINT_NAMES[sn]
            limits = MaicatConfig.JOINT_LIMITS[joint_name]
            self.pwm.setRange(sn, self._convert_radian2pulse(limits[0]), self._convert_radian2pulse(limits[1]))

    def _disconnect(self, sn):
        self.pwm.setTarget(sn, 0)

    def _convert_radian2pulse(self, radian_value):
        target = int(637 * radian_value + 1500) * 4
        target = np.clip(target, 2000, 10000)
        return target # (pi + 2x) / pi * 1000 + 500

if __name__ == '__main__':
    try:
        mr = MaicatReady1()
    except KeyboardInterrupt:
        print("Shutting down - MaicatReady")


