from matplotlib.pyplot import *
from numpy import *

import os, os.path
import random
import string
import cherrypy

def mainloop(self):
    "Interface to cherrypy's mainloop"

    # launch a set of child processes to handle each request
    # within a child process. 
    self.registry.start_processes( self._settings.get('js_pool',20) )

    self._config['global'] = {
        'server.socket_host' : self._settings.get('host',"0.0.0.0"),
        'server.socket_port' : self._settings.get('port',8080),
        'server.thread_pool' : self._settings.get('thread_pool',20)
    }

    sdir = self._settings.get('static_dir',None)
    if sdir:
        self.logger.debug("adding static content directory %s" % sdir)
        self._config["/"]['tools.staticdir.on'] = True
        self._config["/"]['tools.staticdir.dir'] = sdir

    self._config['/favicon.ico']={
        'tools.staticfile.on': len(self._settings.get('favicon','')) > 0,
        'tools.staticfile.filename': self._settings.get('favicon','') 
    }
            
if __name__ == '__main__':
    mainloop()
