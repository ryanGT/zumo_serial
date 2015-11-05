from matplotlib.pyplot import *
from numpy import *

import os, os.path
import random
import string
import cherrypy
import zumo_serial
import time

from myip import myip

class StringGenerator(object):
    def __init__(self):
        self.zumo = None
        self.top_header = """<html>
        <head>
        <link href="/static/css/style.css" rel="stylesheet">
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
        </body>
        </html>"""

    @cherrypy.expose
    def init_PID_ro(self):
        self.zumo = zumo_serial.zumo_serial_pd_control_rotate_only(kp=0.2, \
                                                                   kd=0, \
                                                                   numsensors=6)
        raise cherrypy.HTTPRedirect("/")
        
        
    @cherrypy.expose
    def init_PID(self):
        self.zumo = zumo_serial.zumo_serial_connection_pd_control(kp=0.25, \
                                                                  kd=1, \
                                                                  numsensors=6)
        raise cherrypy.HTTPRedirect("/")


    @cherrypy.expose
    def init_OL(self):
        self.zumo = zumo_serial.zumo_serial_ol_rotate_only()
        raise cherrypy.HTTPRedirect("/")


    @cherrypy.expose
    def OL(self):
        if self.zumo is None:
            self.init_OL()
            
        out = self.top_header + \
              self.header + \
              """<form method="get" action="run_test">
              Pulse Amp:<br>
              <input type="text" name="amp" value="50">
              <br>
              Pulse Width:<br>
              <input type="text" name="width" value="20"><br>
              N:<br>
              <input type="text" name="N" value="200"><br>""" + \
              self.tail
        return out
    

        return "OL Stuff goes here"
    
        
    @cherrypy.expose
    def PID(self):
        if self.zumo is None:
            self.init_PID()
            
        out = self.top_header + \
              self.header + \
              """<form method="get" action="run_test">
              Kp:<br>
              <input type="text" name="Kp" value="0.2">
              <br>
              Ki:<br>
              <input type="text" name="Ki" value="0"><br>
              Kd:<br>
              <input type="text" name="Kd" value="0.7"><br>""" + \
              self.tail
        return out
    

    @cherrypy.expose
    def menu(self):
        out = self.top_header + \
              """<form method="get" action="init_OL">
              <button type="submit">OL</button>
              </form>
              <form method="get" action="init_PID_ro">
              <button type="submit">PID Rotate Only</button>
              </form>"""

        return out

        
    @cherrypy.expose
    def index(self):
        if self.zumo is None:
            raise cherrypy.HTTPRedirect("/menu")
        elif isinstance(self.zumo, zumo_serial.zumo_serial_connection_pd_control):
            raise cherrypy.HTTPRedirect("/PID")
        elif isinstance(self.zumo, zumo_serial.zumo_serial_ol_rotate_only):
            raise cherrypy.HTTPRedirect("/OL")


    def return_with_back_link(self, str_in):
        link1 = '<br><a href="/">back</a>'
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
        msg = self.zumo.open_and_check_serial()
        if msg is not None:
            #we are ready for business
            msg2 = 'version 1.0.0'
            link1 = '<a href="/">back</a>'
            #link2 = '<a href="http://localhost:8080">localhost link</a>'
            #link3 = '<a href="http://192.168.0.111:8080">192 link</a>'
            str_out = '<br>'.join([msg, msg2, link1])
            return str_out

        else:
            start = """<html><head><title><Opening Serial Connection</title>
            <meta http-equiv="refresh" content="1">
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
        return self.show_report()

        
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
        header = """ <html>
        <head>
        <title>CherryPy Test Results</title>
        </head>
        <html>
        <body>"""
        top_part = self.zumo.get_report()
        img_part = """<img src="/img/webtest.png" width=600px>
        <br><a href="/%s">download data</a>
        <br><a href="/">email data to yourself</a>
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
        e = self.zumo.error[0:sn]
        plot(n, e, linewidth=2)
        xlabel('Loop count $n$')
        ylabel('error')
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
