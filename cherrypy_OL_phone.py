import matplotlib
matplotlib.use('Agg')

from matplotlib.pyplot import *
from numpy import *

import os, os.path
import sys

# hack to work around PYTHONPATH issue
myrelpaths = ['git/research','git/krauss_misc','git/bad']
home_dir = '/home/pi'
myabspaths = [os.path.join(home_dir, item) for item in myrelpaths]

for curpath in myabspaths:
    if curpath not in sys.path:
        sys.path.append(curpath)
        

import random
import string
import cherrypy
import zumo_serial
import time

import ipfinder
#myip = ipfinder.get_ip()

#if myip == '0':
#    myip = ipfinder.read_ip_from_txt()

myip = ipfinder.read_ip_from_txt()
    
    
#from myip import myip

class StringGenerator(object):
    def __init__(self):
        self.zumo = None
        self.top_header = """<html>
        <head>
        <link href="/static/css/style.css" rel="stylesheet">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        </head>
        <body>"""
        self.header = """<form method="get" action="open_and_check_serial">
        <button type="submit">Open Serial</button>
        </form>
        <form method="get" action="calibrate">
        <button type="submit">Calibrate</button>
        </form>"""
        self.tail = """<br>
        <button type="submit">Run Test</button>
        </form>
        <br>
        <br>
        <form method="get" action="show_report">
        <button stype="submit">Show Last Report</button>
        </form>
        </body>
        </html>"""

    @cherrypy.expose
    def init_PID_ro(self):
        self.zumo = zumo_serial.zumo_serial_pid_control_rotate_only(kp=0.2, \
                                                                    kd=0, \
                                                                    ki=0, \
                                                                    numsensors=6)
        raise cherrypy.HTTPRedirect("/")
        

    @cherrypy.expose
    def init_arb_tf(self):
        self.zumo = zumo_serial.zumo_arbitrary_TF(numlist=[1,1],denlist=[1,1], gain=1)
        raise cherrypy.HTTPRedirect("/")
    
        
    @cherrypy.expose
    def init_fixed_sine(self):
        self.zumo = zumo_serial.zumo_fixed_sine(kp=0.25, \
                                                N=500, \
                                                amp=500, \
                                                freq=1.0, \
                                                numsensors=6)
        raise cherrypy.HTTPRedirect("/")
    
        
    @cherrypy.expose
    def init_PID(self):
        self.zumo = zumo_serial.zumo_serial_connection_pid_control(kp=0.25, \
                                                                   kd=1, \
                                                                   ki=0, \
                                                                   numsensors=6)
        raise cherrypy.HTTPRedirect("/")


    @cherrypy.expose
    def init_OL(self):
        self.zumo = zumo_serial.zumo_serial_ol_phone()
        #raise cherrypy.HTTPRedirect("/")


    @cherrypy.expose
    def forward(self):
        self.zumo.run_one_burst(400,400,1)
        raise cherrypy.HTTPRedirect("/OL")


    @cherrypy.expose
    def left(self):
        self.zumo.run_one_burst(-400,400,0.1)
        raise cherrypy.HTTPRedirect("/OL")


    @cherrypy.expose
    def right(self):
        self.zumo.run_one_burst(400,-400,0.1)
        raise cherrypy.HTTPRedirect("/OL")


    @cherrypy.expose
    def back(self):
        self.zumo.run_one_burst(-400,-400,1)
        raise cherrypy.HTTPRedirect("/OL")


    @cherrypy.expose
    def OL(self):
        if self.zumo is None:
            self.init_OL()
            
        out = self.top_header + \
              """<form method="get" action="forward">
              <button type="submit" style="font-size:40px;min-width: 200px; width:300px;">Forward</button>
              </form>
              <form method="get" action="left">
              <button type="submit" style="font-size:40px;min-width: 200px;width: 300px;">Left</button>
              </form>
              <form method="get" action="right">
              <button type="submit" style="font-size:40px;min-width: 200px; width:300px;">Right</button>
              </form>
              <form method="get" action="back">
              <button type="submit" style="font-size:40px;min-width: 200px; width:300px;">Back</button>
              </form>"""
        return out
    

        
    @cherrypy.expose
    def PID(self):
        if self.zumo is None:
            self.init_PID()

        Kp = self.zumo.kp
        Ki = self.zumo.ki
        Kd = self.zumo.kd
        N = self.zumo.N
        zmin = self.zumo.min
        nom = self.zumo.nominal_speed

        middle ="""<form method="get" action="run_test">
            Kp:<br>
            <input type="text" name="Kp" value="%0.4g">
            <br>
            Ki:<br>
            <input type="text" name="Ki" value="%0.4g"><br>
            Kd:<br>
            <input type="text" name="Kd" value="%0.4g"><br>
            N:<br>
            <input type="text" name="N" value="%i"><br>
            min:<br>
            <input type="text" name="min" value="%i"><br>
            nominal_speed:<br>
            <input type="text" name="nominal_speed" value="%i"><br>
            """ % (Kp,Ki,Kd,N,zmin,nom)

        out = self.top_header + \
              self.header + \
              middle + \
              self.tail
        return out

    @cherrypy.expose
    def arb_tf(self):
        if self.zumo is None:
            self.init_arb_tf()

        numlist = self.zumo.numlist
        denlist = self.zumo.denlist
        numstrlist = [str(item) for item in numlist]
        denstrlist = [str(item) for item in denlist]
        gain = self.zumo.gain
        numstr = ', '.join(numstrlist)
        denstr = ', '.join(denstrlist)
        N = self.zumo.N
        zmin = self.zumo.min
        nom = self.zumo.nominal_speed

        middle ="""Arb TF <br>
            <form method="get" action="run_test">
            numerator list:<br>
            <input type="text" name="numstr" value="%s">
            <br>
            denominator list:<br>
            <input type="text" name="denstr" value="%s"><br>
            gain:<br>
            <input type="text" name="gain" value="%0.4g"><br>
            N:<br>
            <input type="text" name="N" value="%i"><br>
            min:<br>
            <input type="text" name="min" value="%i"><br>
            nominal_speed:<br>
            <input type="text" name="nominal_speed" value="%i"><br>
            """ % (numstr, denstr, gain, N, zmin, nom)

        out = self.top_header + \
              self.header + \
              middle + \
              self.tail
        return out

    
    @cherrypy.expose
    def fixed_sine(self):
        if self.zumo is None:
            self.init_fixed_sine()

        Kp = self.zumo.kp
        N = self.zumo.N
        amp = self.zumo.amp
        freq = self.zumo.freq

        middle ="""<form method="get" action="run_test">
            Kp:<br>
            <input type="text" name="Kp" value="%0.4g">
            <br>
            amp:<br>
            <input type="text" name="amp" value="%i"><br>
            freq:<br>
            <input type="text" name="freq" value="%0.4g"><br>
            N:<br>
            <input type="text" name="N" value="%i"><br>
            """ % (Kp,amp,freq,N)

        out = self.top_header + \
              self.header + \
              middle + \
              self.tail
        return out


    @cherrypy.expose
    def serial_connect(self):
        out = self.top_header + \
              """<form method="get" action="open_and_check_serial">
              <button type="submit" style="font-size:40px;min-width: 200px; width:300px;">Open Serial</button>
              </form>"""

        return out

        
    @cherrypy.expose
    def index(self):
        if self.zumo is None:
            raise cherrypy.HTTPRedirect("/serial_connect")
        else:
            raise cherrypy.HTTPRedirect("/OL")


    def return_with_back_link(self, str_in):
        link1 = '<br><a href="/" style="font-size: 60px;">back</a>'
        str_out = str_in + link1
        return str_out

        
    @cherrypy.expose
    def calibrate(self):
        self.zumo.calibrate()
        return self.return_with_back_link("calibration complete")

    @cherrypy.expose
    def open_serial_msg(self):
        msg = """<html><head><title>Serial connection</title>
        <script type="text/javascript">
        setTimeout(1000, "document.location = '/open_serial'");
        </script>
        </head>
        <body>
        opening the serial connection to the Arduino.  Please standby.<br>
        <a href="/open_serial">Click here</a> if you are not redirected in 5 seconds.
        </body></html>"""
        return msg


    @cherrypy.expose
    def open_and_check_serial(self):
        print('')
        print('in open_and_check_serial')
        print('')
        if self.zumo is None:
            print('before init')
            self.init_OL()
            print('after init')

        print('')
        msg = self.zumo.open_and_check_serial()
        print('msg = %s' % msg)
        print('')
        hdr = """<html>
            <head>
            <link href="/static/css/style.css" rel="stylesheet">
            <meta name="viewport" content="width=device-width, initial-scale=1">
            </head>
            <body>"""
        tail = """</body></html>"""
        if msg is not None:
            #we are ready for business
            msg2 = 'version 1.0.0'
            link1 = '<a href="/" style="font-size: 60px;">back</a>'
            #link2 = '<a href="http://localhost:8080">localhost link</a>'
            #link3 = '<a href="http://192.168.0.111:8080">192 link</a>'
            str_out = '<br>'.join([hdr,msg, msg2, link1,tail])
            return str_out

        else:
            start = """<html><head><title><Opening Serial Connection</title>
            <meta http-equiv="refresh" content="1">
            <link href="/static/css/style.css" rel="stylesheet">
            <meta name="viewport" content="width=device-width, initial-scale=1">
            </head>
            <body>"""
            line1 = 'Opening serial connection, please standby<br>'
            wait_msg = 'Count = %i' % self.zumo.ser_check_count
            end = "</body></html>"
            return '\n'.join([start,line1,wait_msg,end])
        

    @cherrypy.expose
    def open_serial(self):
        msg = self.zumo.open_serial()
        msg2 = 'version 1.0.0'
        link1 = '<a href="/">back</a>'
        #link2 = '<a href="http://localhost:8080">localhost link</a>'
        #link3 = '<a href="http://192.168.0.111:8080">192 link</a>'
        str_out = '<br>'.join([msg, msg2, link1])
        return str_out


    @cherrypy.expose
    def save_csv_and_png(self):
        self.zumo.save(os.path.join('data','webtest'))#<-- probably need a data folder eventually
        self.save_plot()
        

    @cherrypy.expose
    def run_test(self, **kwargs):
        self.zumo.parse_args(**kwargs)
        self.zumo.run_test()
        self.save_csv_and_png()
        raise cherrypy.HTTPRedirect("/show_report")
        #return self.show_report()

        
    @cherrypy.expose
    def kraussfunc(self, **kwargs):
        str_out = ''
        first = 1
        for key, val in kwargs.iteritems():
            if first:
                first = 0
            else:
                str_out += ', '
            str_out += '%s:%s' % (key, val)
        self.zumo.kp = float(kwargs['Kp'])
        self.zumo.kd = float(kwargs['Kd'])
        self.zumo.run_test(N=500)
        self.zumo.save('webtest')#<-- probably need a data folder eventually
        #return cherrypy.lib.static.serve_file(os.path.abspath(self.zumo.data_file_name))
        #return str_out
        self.save_plot()
        return self.showimage()
    
    
    @cherrypy.expose
    def generate(self, length=8):
        some_string = ''.join(random.sample(string.hexdigits, int(length)))
        cherrypy.session['mystring'] = some_string
        return some_string

    @cherrypy.expose
    def display(self):
        return cherrypy.session['mystring']


    @cherrypy.expose 
    def showimage(self): 
        cherrypy.response.headers['Content-Type']= "image/png" 
        f = open("img/webtest.png", "rb") 
        contents = f.read() 
        f.close() 
        return contents 


    @cherrypy.expose
    def show_report(self):
        #        <br><a href="/">email data to yourself</a>
        header = """ <html>
        <head>
        <title>CherryPy Test Results</title>
        </head>
        <html>
        <body>"""
        top_part = self.zumo.get_report()
        img_part = """<img src="/img/webtest.png" width=600px>
        <br><a href="/%s">download data</a>
        <br><a href="/">back</a>
        </body>
        </html>""" % self.zumo.data_file_name
        out = " <br> ".join([header, top_part, img_part])
        return out


    @cherrypy.expose
    def show_plot_with_links(self):
        line1 = "Kp = %0.4g, Ki = %0.4g, Kd = %0.4g" % (self.zumo.kp, \
                                                        self.zumo.ki, \
                                                        self.zumo.kd)
        line2 = "Lap time = %0.5g" % self.zumo.laptime
        line3 = "Total error = %0.6g" % self.zumo.total_e
        # report should include total error through stopn 
        header = """ <html>
        <head>
        <title>CherryPy Test Results</title>
        </head>
        <html>
        <body>"""

        img_part = """<img src="/img/webtest.png" width=600px>
        <br><a href="/">download data</a>
        <br><a href="/">email data to yourself</a>
        <br><a href="/">back</a>
        </body>
        </html>"""
        out = " <br> ".join([line1, line2, line3, img_part])
        return out

        
    def save_plot(self):
        self.pngname = "img/webtest.png"
        ioff()
        figure(1)
        clf()
        sn = self.zumo.stopn
        n = self.zumo.nvect[0:sn]
        if hasattr(self.zumo, 'vdiff_vect'):
            v = self.zumo.vdiff_vect[0:sn]
            plot(n,v, linewidth=2)
        e = self.zumo.error[0:sn]
        plot(n, e, linewidth=2)
        xlabel('Loop count $n$')
        ylabel('error')
        if hasattr(self.zumo, 'vdiff_vect'):
            legend(['vdiff','line sensor'])
        savefig(self.pngname, dpi=100) 
        

    @cherrypy.expose 
    def plot(self): 
        _header = """ 
            <html> 
            <head> 
            <title>Random notes</<title> 
            <link rel="stylesheet" type="text/css" href="/style.css"></link> 
            </head> 
            <body> 
            <div class="container">""" 
        _footer = """ 
            </div> 
            </body> 
            </html>""" 
        ioff()
        figure(1)
        clf()
        plot(self.zumo.nvect, self.zumo.error)
        savefig("img/webtest.png", dpi=150) 
        cherrypy.response.headers['Content-Type']= 'text/html' 
        page = [_header] 
        page.append('<img src="/img/webtest.png" width="1200" height="800" />' ) 
        page.append(_footer) 
        return page
            
if __name__ == '__main__':
     # make subdirectories if needed
     dirlist = ['public','img','data']
     for item in dirlist:
         if not os.path.exists(item):
             os.mkdir(item)
             
     conf = {
         '/': {
             'tools.sessions.on': True, \
             'tools.staticdir.root': os.path.abspath(os.getcwd()), \
             }, \
         '/static': {
             'tools.staticdir.on': True,
             'tools.staticdir.dir': './public'
             }, \
         '/img': {
             "tools.staticdir.on": True,
             "tools.staticdir.dir": './img',
             }, \
         '/data': {
             "tools.staticdir.on": True,
             "tools.staticdir.dir": './data',
             }
         }
     cherrypy.config.update(conf)
     cherrypy.config.update({'server.socket_host':myip})
     cherrypy.quickstart(StringGenerator(), '/', conf)
     #cherrypy.quickstart()
