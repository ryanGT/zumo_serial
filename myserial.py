import serial_utils

portname = '/dev/tty.usbmodem1421'

ser = serial_utils.Open_Serial(portname)
ser.flushInput()
ser.flushOutput()
