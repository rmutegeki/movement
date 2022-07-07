#!/usr/bin/env python
'''
Created on 2022. 1. 12.
@author: macroact
'''

import RPi.GPIO as GPIO
import time

try:
    pin = 12
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.IN)
    while True:
        read = GPIO.input(pin)
        val = read
        print('voltage : %d ' %val)
        time.sleep(0.5)

finally:
    GPIO.cleanup()

