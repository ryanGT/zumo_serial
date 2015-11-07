#!/usr/bin/env python

#myip = '192.168.0.106'
#myip = '192.168.2.7'
import ipfinder
import time
ipaddr = ipfinder.get_ip()
stamp = "#" + time.strftime('%m/%d/%Y %H:%M:%S%p')
filepath = '/home/pi/zumo_serial/ip.txt'
f = open(filepath,'w')
f.write(stamp)
f.write('\n')
f.write(ipaddr)
f.close()

