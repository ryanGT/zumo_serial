#!/usr/bin/env python

#myip = '192.168.0.106'
#myip = '192.168.2.7'
import ipfinder
import time
ipaddr = ipfinder.get_ip()
valid_ips = ['10.10','192.168']
tries = 0

def check_ip(str_in):
    for ip in valid_ips:
        if str_in.find(ip) == 0:
            return True
        
    return False

while not check_ip(ipaddr):
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
if ipaddr.find('10.10') == 0:
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

