'''
.. module:: optitrack
   :synopsis: Module for handling and retrieving OptiTrack positioning data.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    Module for handling and retrieving OptiTrack positioning data.
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

import socket
import struct
import sys
import time

from numpy import array, mean, rad2deg, zeros
from numpy.linalg import inv
from numpy import matrix

from util import getLocalIP
from util import rot2ZYXeuler, quaternion2rot
from parseNatNet import RigidBody, parseNatNet



class OptiTrackInterface(object):
    '''
    classdocs
    '''
    
    def __init__(self, localIP=None, buffersize=2**12,
                 multicastIP='239.255.42.99', multicastPort=1511):
        '''
        Constructor
        '''
        self.__multicastIP = multicastIP
        if localIP:
            self.__localIP = localIP
        else:
            self.__localIP = getLocalIP()
#         print self.__localIP
        
        self.__port = multicastPort
        self.__buffer = bytearray(buffersize)
        self.buffersize = buffersize
        
        # TODO: define tracked objects and origin
        self.trackedObjects = [1]
        self.origin = RigidBody()
        
        self.setup()
    
    def __del__(self):
        '''
        Destructor -- closes the socket
        '''
        self.__socket.close()
        
    def _close(self): # FIXME: delete this --- temp solution
        self.__socket.close()
    
    def setup(self):
        """
        Creates a socket, sets the necessary options on it, then binds it.
        """
         
        # create a UDP socket
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
           
        # allow reuse of addresses
        self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # request membership to multicast group
        if sys.platform.startswith('linux'):
            msgReq = struct.pack("4sl", socket.inet_aton(self.__multicastIP),
                                 socket.INADDR_ANY)
        else:
            msgReq = struct.pack("4s4s", socket.inet_aton(self.__multicastIP),
                                 socket.inet_aton(self.__localIP))
         
        self.__socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                 msgReq)

        # bind socket to multicast port
        # NOTE: for Linux Ubuntu/Mac you have to bind to all accessible addresses in
        # order to receive something
        # TODO: test on Windows -- it seems on Windows you have bind to actual
        # local address to receive something
        if sys.platform.startswith('win32'):
            self.__socket.bind((self.__localIP, self.__port))
        else:
            self.__socket.bind(('', self.__port))
        
#-------------------------------------------------------------------------------

    def trackInfo(self):
        '''
        Note: only for ZYX Euler angles
        '''
        # Receive the data into a buffer
#         self.__socket.recv_into(self.__buffer, self.buffersize)
#         print ' '.join(map(hex, self.__buffer))
        msg = self.__socket.recv(4096)
#         print len(msg)
#         print ' '.join(map(hex, bytearray(msg)))
        
#         Parse data 
        markerSets, rigidBodies = parseNatNet(msg)
#         markerSets, rigidBodies = parseNatNet(self.__buffer)
#         
#         for rigidBodyID in self.trackedObjects:
#             rigidBody = rigidBodies[rigidBodyID] 
#             # Check if trackable is seen
#             if not any(rigidBody.SE3):
#                 return None
#             
        # print OptiTrack data -- TODO: recode this with logging library
        if False:
            print '--------------------------------------------'
            print 'Real time info in OptiTrack system (m, deg):'
               
            # the same data displayed in the Motive/OptiTrack panel
            for rigidBodyID in self.trackedObjects:
                rigidBody = rigidBodies[rigidBodyID]
#                 rigidBody.SE3[3:] = array(list([rigidBody.SE3[6], rigidBody.SE3[3], rigidBody.SE3[5], rigidBody.SE3[4]])) # + list(rigidBody.SE3[3:6]))
                print rigidBody.SE3[3:]
                print quaternion2rot(rigidBody.SE3[3:])
                euler = rot2ZYXeuler(quaternion2rot(rigidBody.SE3[3:])) # roll, pitch, yaw
                print euler
                print rigidBodyID, '->', rigidBody.SE3[:3], map(rad2deg, euler)
          
        return rigidBodies

#-------------------------------------------------------------------------------
#     def globalOriginPositionInit(self):
#         # function [frame, invTstep] = glblOrgnPosInit(opti, dispCtrl, frame, K500)
#         initStep = 200
# 
#         print '----------------------------------------------------------------'
#         print 'Global system origin initialization ......'
# 
#         count50 = 0
# 
#         # to find the index of the trackable in opti.trackedObjectID
# #         for i in range(opti.trackedObjectNum):
# #             if frame.OrgnBy == opti.trackedObjectID(i):
# #                 globalOriginID = i
# #                 break
#         
# 
#         # collecting enough data for initilization -----------------------------
#         t0 = time.time()
#         timeSteps = []
#         infoOrigin = zeros(initStep, 7)
#         for i in range(initStep):
#             tc = time.time()
#             timeSteps.append(tc - t0)
#             rigidBodies = self.trackInfo() # expressed in (o)
#             infoOrigin[i, :] = rigidBodies[self.origin.ID] 
#             
#             count50 += 1;
#             print '.',
#             if count50 >= 50:
#                 print
#                 count50 = 0
#             t0 = tc
# 
#         self.origin.SE3 = mean(infoOrigin, axis=1) # expressed in (oP), unit: m
#         self.origin.q2 = -self.origin.q2
# 
# #         TODO:
# #         mx, my, mz, q0, q1, q2, q3 = mean(infoOrigin, axis=1)
# #         
# #         self.origin.SE3[:3] = [mx, my, mz] # expressed in (oP), unit: m
# #         
# #         q2 = -q2;          # modified.
# #         
# #         # TODO: see if this is important???
# #         # Rotation matrix from global fixed system to the global system
# #         C_gFg = matrix(quaternion2rot([q0, q1, q2, q3]))
# #         invC_gFg = inv(C_gFg)
# #         
# #         # rotation combination
# #         C_oPg = C_oPgF * C_gFg
# #         invC_oPg = inv(C_oPg)
# #         
# #         # center of mass offset to the center of the trackable/rigid body.
# #         #TODO: see what these is all about!!!
# #         self.origin += C_oPgF * C_gFg * ( -K500.track)
# #         
# #         print 'average time: ', str(mean(timeSteps)*1000), 'ms'
# #         timeStep = mean(timeSteps)   # in second
# #         invTstep = 1/timeStep


if __name__ == '__main__':
    #test OptiTrack interface
    N=10
    
    opti = OptiTrackInterface()
    for i in range(N):
        print 'Fetching package:', i+1
        opti.trackInfo()
