import serial
ser = serial.Serial("/dev/ttyACM0", 9600)

while (1==1):
   data=ser.readline()
   print(data)

