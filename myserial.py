import serial_utils

portname = '/dev/ttyACM0'
#portname = '/dev/ttyACM1'

ser = serial_utils.Open_Serial(portname)
ser.flushInput()
ser.flushOutput()
