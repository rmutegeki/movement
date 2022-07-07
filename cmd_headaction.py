import serial
import argparse

parser = argparse.ArgumentParser(description='')
parser.add_argument('-a', required=True, help='head action 1~4 / ')
args = parser.parse_args()
cmd = 'a,' + str(args.a)
#print (cmd)
ser = serial.Serial("/dev/ttyACM0", 9600)
ser.write(cmd)

