import socket
import os
#import time

def get_ip():
	try:
		gw = os.popen("ip -4 route show default").read().split()
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.connect((gw[2], 0))
		ipaddr = s.getsockname()[0]
		gateway = gw[2]
		host = socket.gethostname()
		myip = ipaddr
	except:
		myip='0'
	
	return myip


def read_ip_from_txt():
	f = open('ip.txt','r')
	lines = f.readlines()
	
	# assumefirst non-blank line that doesn't start with # contains ip
	lineout = ''
	
	for line in lines:
		clean_line = lines.strip()
		if clean_line[0] != '#':
			lineout = clean_line
			
	return lineout
	

