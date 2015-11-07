#!/usr/bin/env python

#myip = '192.168.0.106'
#myip = '192.168.2.7'
import ipfinder
import time
ipaddr = ipfinder.get_ip()

tries = 0
print('ipaddr = %s' % ipaddr)

while ipaddr.find('192.168') != 0:
    tries += 1
    print('tries = %i' % tries)
    time.sleep(1)
    ipaddr = ipfinder.get_ip()
    if tries > 30:
        break
    

stamp = "#" + time.strftime('%m/%d/%Y %H:%M:%S%p')
filepath = '/home/pi/zumo_serial/ip.txt'
f = open(filepath,'w')
f.write(stamp)
f.write('\n')
f.write(ipaddr)
f.close()

