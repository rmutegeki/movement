import serial
ser = serial.Serial("/dev/ttyACM0", 9600)
ser.write('a')

while (1==1):

   data=ser.readline()
   print(data)

