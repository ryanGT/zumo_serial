import serial_utils
import os

pat = '/dev/ttyACM%i'

found_it = False

for i in range(10):
    portname = pat % i
    if os.path.exists(portname):
        found_it = True
        break

#portname = '/dev/ttyACM1'
#portname = '/dev/cu.usbmodem1411'

if found_it:
    ser = serial_utils.Open_Serial(portname)
    ser.flushInput()
    ser.flushOutput()
