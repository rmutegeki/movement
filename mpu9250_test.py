import FaBo9Axis_MPU9250
import time
import sys

mpu9250 = FaBo9Axis_MPU9250.MPU9250()

try:
    while True:
        accel = mpu9250.readAccel()
        print " ax = " , ( accel['x'] ), " ay = " , ( accel['y'] ), " az = " , ( accel['z'] )

        gyro = mpu9250.readGyro()
#        print " gx = " , ( gyro['x'] ), " gy = " , ( gyro['y'] ), " gz = " , ( gyro['z'] )

        mag = mpu9250.readMagnet()
#        print " mx = " , ( mag['x'] ), " my = " , ( mag['y'] ), " mz = " , ( mag['z'] )
#        print

        time.sleep(1)

except KeyboardInterrupt:
    sys.exit()
