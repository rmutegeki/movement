import serial
import time
#import argparse
#parser = argparse.ArgumentParser(description='')
#parser.add_argument('-a', required=True, help='head action 1~4 / ')
#args = parser.parse_args()
#cmd = 'a,' + str(args.a)
#print (cmd)
ser = serial.Serial("/dev/ttyACM0", 9600)
#volum 30
ser.write('v,30')
ser.write('a,2')

while (1==1):
    data = ser.readline()
    result = data[0:2]
    print (data)
    if (result == 'T1'): 
       print ('CHECK ... T1') 
       ser.write('a,4')
       time.sleep(2)

    if (result == 'T2'): 
       print ('CHECK ... T2') 
       ser.write('a,3')
       time.sleep(2)

    if (result == 'T3'): 
       print ('CHECK ... T3') 
       ser.write('a,2')
       time.sleep(2)

