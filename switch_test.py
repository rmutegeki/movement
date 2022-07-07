import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Foot switch
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(8, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Touch switch (10/body, 9/head back   11/head front) 
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(9, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


try:
	while True:
	   if (GPIO.input(23) == 1): print "front left"
	   if (GPIO.input(24) == 1): print "front right"
	   if (GPIO.input(25) == 1): print "rear left"
	   if (GPIO.input(8) == 1): print "rear right"

	   if GPIO.input(10) == 1: print "touch body"

	   time.sleep(0.5)
#	   a = input()
except KeyboardInterrupt:
	GPIO.cleanup()
