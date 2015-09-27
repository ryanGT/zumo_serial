from matplotlib.pyplot import *
from numpy import *

import os, os.path
import random
import string
import cherrypy
import zumo_serial

class StringGenerator(object):
    def __init__(self):
        self.zumo = zumo_serial.zumo_serial_connection_pd_control(kp=0.25, \
                                                                  kd=1)
        
    @cherrypy.expose
    def index(self):
        return """<html>
        <head>
        <link href="/static/css/style.css" rel="stylesheet">
        </head>
        <body>
        <form method="get" action="open_serial">
        <button type="submit">Open Serial</button>
        </form>
        <form method="get" action="calibrate">
        <button type="submit">Calibrate</button>
        </form>
        <form method="get" action="kraussfunc">
        Kp:<br>
        <input type="text" name="Kp" value="0.2">
        <br>
        Ki:<br>
        <input type="text" name="Ki" value="0"><br>
        Kd:<br>
        <input type="text" name="Kd" value="0.7"><br>
        <br>
        <button type="submit">Run Test</button>
        </form>
        </body>
        </html>"""

    def return_with_back_link(self, str_in):
        link1 = '<br><a href="/">back</a>'
        str_out = str_in + link1
        return str_out
        
    @cherrypy.expose
    def calibrate(self):
        self.zumo.calibrate()
        return self.return_with_back_link("calibrating complete")


    @cherrypy.expose
    def open_serial(self):
        msg = self.zumo.open_serial()
        link1 = '<br><a href="/">back</a>'
        #link2 = '<a href="http://localhost:8080">localhost link</a>'
        #link3 = '<a href="http://192.168.0.111:8080">192 link</a>'
        str_out = '\n'.join([msg, link1])
        return str_out

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
    def show_plot_with_links(self):
        line1 = "Kp = %0.4g, Ki = %0.4g, Kd = %0.4g" % (self.zumo.kp, \
                                                        self.zumo.ki, \
                                                        self.zumo.kd)
        line2 = "Lap time = %0.5g" % self.zumo.laptime
        line3 = "Total error = %0.6g" % self.zumo.total_e
        # report should include total error through stopn 
        header = """ <html>
        <head>
        <title>CherryPy Test Restuls</title>
        </head>
        <html>
        <body>"""

        img_part = """<img src="/img/webtest.png">
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
     conf = {
         '/': {
             'tools.sessions.on': True, \
             'tools.staticdir.root': os.path.abspath(os.getcwd()), \
             },
         '/static': {
             'tools.staticdir.on': True,
             'tools.staticdir.dir': './public'
             },
         '/img': {
			 "tools.staticdir.on": True,
             "tools.staticdir.dir": './img',
             }
         }
     cherrypy.config.update(conf)
     cherrypy.config.update({'server.socket_host':'192.168.0.111'})
     cherrypy.quickstart(StringGenerator(), '/', conf)
     #cherrypy.quickstart()
