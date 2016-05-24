'''
    #TODO:
    Copyright (C) 2013  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Laboratory, Boston University

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import SocketServer, StringIO

from numpy import loadtxt


class GUIDataHandler(SocketServer.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """
        
    def handle(self):
        # self.request is the TCP socket connected to the client
        
        self.data = self.request.recv(4096).strip()
        print "{} wrote:".format(self.client_address[0])
        print self.data
        print
        
        self.server.waypoints = loadtxt(StringIO.StringIO(self.data)).reshape(-1, 2)

def getWaypointsFromGUI(host, port):
    # create the server, binding to localhost on port 9999
    server = SocketServer.TCPServer((host, port), GUIDataHandler)
    server.handle_request()

    # return coordinates
    return server.waypoints

if __name__ == '__main__':
    getWaypointsFromGUI('localhost', 9000)