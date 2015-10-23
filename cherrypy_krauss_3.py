import cherrypy
class HelloWorld(object):
    def index(self):
        return "Hello World!"
    index.exposed = True

# bind to all IPv4 interfaces
cherrypy.config.update({'server.socket_host': '0.0.0.0'})
cherrypy.quickstart(HelloWorld())
