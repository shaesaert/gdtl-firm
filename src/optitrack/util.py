'''
.. module:: util
   :synopsis: Utility functions for handling OptiTrack data.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    Utility functions for handling OptiTrack data.
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

import re
import socket

from numpy import  array, eye
from numpy import arcsin, arctan2
from numpy.linalg import norm


#-------------------------------------------------------------------------------
# network utility functions

def isIPLocal(ip_string):
    """
    Uses a regex to determine if the input ip is on a local network. Returns a boolean.
    """
    combined_regex = "(^10\.)|(^172\.1[6-9]\.)|(^172\.2[0-9]\.)|(^172\.3[0-1]\.)|(^192\.168\.)"
    return re.match(combined_regex, ip_string) is not None
 
def getLocalIP():
    """
    Returns the first externally facing local IP address that it can find or the
    loopback address in case of failure.
    """
    
    # get IPs from the data list returned by socket.getaddrinfo
    localIPs = [x[4][0] for x in socket.getaddrinfo(socket.gethostname(), 80)
                 if isIPLocal(x[4][0])]
    
    # return the first IP
    if localIPs:
        return localIPs[0]
      
    # let the OS figure out which interface to use
    # create a standard UDP socket
    tempSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # connect to one of Google's DNS servers
        tempSocket.connect(('8.8.8.8', 9))
        # get the interface used by the socket
        localIP = tempSocket.getsockname()[0]
    except socket.error:
        # return loopback address 127.0.0.1 if connection fails
        localIP = "127.0.0.1"
    finally:
        # close temporary socket
        tempSocket.close()
    return localIP


#-------------------------------------------------------------------------------
# Geometry utility functions

def quaternion2rot(q):
    '''
    Transforms a unit quaternion into a rotation matrix. 
    See: http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
    for longer discussion.
    The formula is:
    
        R = 1-2*(q2^2+q3^2)     2*(q1*q2-q0*q3)   2*(q0*q2+q1*q3)
              2*(q1*q2+q0*q3) 1-2*(q1^2+q3^2)     2*(q2*q3-q0*q1)
              2*(q1*q3-q0*q2)   2*(q0*q1+q2*q3) 1-2*(q1^2+q2^2)
    
    '''
    if norm(q) - 1 > 1e-2:
        raise ValueError('Not a unit quaternion!')
        #return  eye(3)
    
    q0, q1, q2, q3 = q
    return array([[1-2*(q2**2+q3**2),   2*(q1*q2-q0*q3),   2*(q0*q2+q1*q3)],
                  [  2*(q1*q2+q0*q3), 1-2*(q1**2+q3**2),   2*(q2*q3-q0*q1)],
                  [  2*(q1*q3-q0*q2),   2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2)]]) 

def rot2ZYXeuler(R):
    '''
    Transforms a rotation matrix to the ZYX Tait-Bryan/Euler angles
    representation.
    See: http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
    for longer discussion.
    
        R =  cos(psi)*cos(the), cos(psi)*sin(phi)*sin(the) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(the)
             cos(the)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(the), cos(phi)*sin(psi)*sin(the) - cos(psi)*sin(phi)
            -sin(the),                              cos(the)*sin(phi),                              cos(phi)*cos(the)    
    '''
    return array((arctan2(R[2, 1], R[2, 2]),  # phi -- roll
                  arcsin(-R[2, 0]),           # theta -- pitch
                  arctan2(R[1, 0], R[0, 0]))) # psi -- yaw

#-------------------------------------------------------------------------------

def oneSecRest(opti): # TODO: sort this out
    print '#-------------------------------------------------------------------'
    print 'Wait for one second ......'
    
    for _ in range(150):
        opti.trackInfo(opti)


if __name__ == '__main__':
    pass