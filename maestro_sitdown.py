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


class MaicatSitDown:

    def __init__(self):
        self.pwm = maestro.Controller()
        time.sleep(1)

        # Ready
        self._set_smooth()
        self._sit()
        self.pwm.close()

    def _sit(self):
        for sn in range(18):
            joint_name = MaicatConfig.JOINT_NAMES[sn]
            zero = MaicatConfig.JOINT_READY[joint_name]
            self.pwm.setTarget(sn, self._convert_radian2pulse(zero))
        time.sleep(1)

        for sn in range(18):
            joint_name = MaicatConfig.JOINT_NAMES[sn]
            sit = MaicatConfig.JOINT_SIT[joint_name]
            self.pwm.setTarget(sn, self._convert_radian2pulse(sit))
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

    def _disconnect(self, sn):
        self.pwm.setTarget(sn, 0)

    def _convert_radian2pulse(self, radian_value):
        target = int(637 * radian_value + 1500) * 4
        target = np.clip(target, 3000, 9000)
        return target # (pi + 2x) / pi * 1000 + 500

if __name__ == '__main__':
    try:
        msd = MaicatSitDown()
    except KeyboardInterrupt:
        print("Shutting down - MaicatSitDown")


