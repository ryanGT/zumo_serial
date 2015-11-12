#!/usr/bin/env python

#myip = '192.168.0.106'
#myip = '192.168.2.7'
import ipfinder
import time
ipaddr = ipfinder.get_ip()

tries = 0

while ipaddr.find('192.168') != 0:
    tries += 1
    print('-------------------------')
    print('tries = %i' % tries)
    print('ipaddr = %s' % ipaddr)
    time.sleep(1)
    ipaddr = ipfinder.get_ip()
    if tries > 30:
        print('exiting')
        break
stamp = '#failed'

print('-------------------------')
if ipaddr.find('192.168') == 0:
    print('success:')
    print('ipaddr = %s' % ipaddr)

    stamp = "#" + time.strftime('%m/%d/%Y %H:%M:%S%p')
    print(stamp)

filepath = '/home/pi/zumo_serial/ip.txt'
f = open(filepath,'w')
f.write(stamp)
f.write('\n')
f.write(ipaddr)
f.close()

