import serial
import argparse

parser = argparse.ArgumentParser(description='')
parser.add_argument('-v', required=True, help='volum 0 ~ 30')
args = parser.parse_args()
cmd = 'v,' + str(args.v)
#print (cmd)
ser = serial.Serial("/dev/ttyACM0", 9600)
ser.write(cmd)

