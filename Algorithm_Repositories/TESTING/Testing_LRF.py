#!/usr/bin/env python
import serial
import time
####MAKE SURE LRF is already TURNED ON and has already sent the initial sequence before running this script
#open serial port
ser= serial.Serial('/dev/ttyUSB0',baudrate = 38400, timeout = 1)
#turn LRF on for calibration purposes
#cmd = "LO\r\n"
#ser.write(cmd.encode("ascii"))
#flush input buffer
ser.reset_input_buffer()
#start distance tracking
#cmd = "DT\r\n"
#ser.write(cmd.encode("ascii"))
#create TRUTH distance file
f = open('truth_distance.txt','w')
f.write('distance       time \n')


while 1:
	try:
		cmd = "DM\r\n"
		ser.write(cmd.encode("ascii"))
		ts = time.time()
		line = ser.readline()
		print (line)
		f.write("%s        %f\n" %( line[0:7], ts ))
	except:
		ser.close()
		f.close()
		break


#GPIO.output(24,0)
#GPIO.cleanup()
ser.close()
#f.close()
