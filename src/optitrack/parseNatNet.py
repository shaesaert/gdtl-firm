'''
.. module:: parseNatNet
   :synopsis: Parser for the NatNet v2.0 packages.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    Parser for the NatNet v2.0 packages.
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

import struct

from numpy import array, zeros, rad2deg

from util import rot2ZYXeuler, quaternion2rot


class RigidBody(object):
    '''
    classdocs
    '''
    def __init__(self, ID=0, SE3=None, rigidMarkers=None, meanError=0):
        self.ID = ID
        if SE3 and len(SE3) == 7:
            self.SE3 = array(SE3)
        else:
            self.SE3 = zeros((7,))
        if rigidMarkers:
            self.rigidMarkers = dict(rigidMarkers)
        else:
            self.rigidMarkers = {}
        self.meanError = meanError
        
        self.roll, self.pitch, self.yaw = map(rad2deg,
                                     rot2ZYXeuler(quaternion2rot(self.SE3[3:])))
    
    @property
    def x(self):
        """x coordinate."""
        return self.SE3[0]
    @x.setter
    def x(self, value):
        self.SE3[0] = value
    
    @property
    def y(self):
        """y coordinate."""
        return self.SE3[1]
    @y.setter
    def y(self, value):
        self.SE3[1] = value
        
    @property
    def z(self):
        """z coordinate."""
        return self.SE3[2]
    @z.setter
    def z(self, value):
        self.SE3[2] = value
    
    @property
    def q0(self):
        """q0 coordinate."""
        return self.SE3[3]
    @q0.setter
    def q0(self, value):
        self.SE3[3] = value
        
    @property
    def q1(self):
        """q1 coordinate."""
        return self.SE3[4]
    @q1.setter
    def q1(self, value):
        self.SE3[4] = value
    
    @property
    def q2(self):
        """q2 coordinate."""
        return self.SE3[5]
    @q2.setter
    def q2(self, value):
        self.SE3[5] = value
    
    @property
    def q3(self):
        """q3 coordinate."""
        return self.SE3[6]
    @q3.setter
    def q3(self, value):
        self.SE3[6] = value
    
# TODO: move prints to logging
def parseNatNet(data):
    '''
    Parser function for the NatNet protocol.
    '''
    # Reset data index
    dataIdx = 0
    
    # Message ID and length
    msgId, msgLength = struct.unpack_from('<hh', data, dataIdx)
    dataIdx += struct.calcsize('!hh')
#     print msgId, msgLength
    
    if msgId != 7:
        #OUT OF LAZINESS, I'M NOT DECODING HOW TO PARSE MESSAGE ID 5.
        raise Exception('Message ID ~=7, instead is ' + str(msgId) + '.')
    if msgLength > len(data):
        raise Exception('Message is ' + str(msgLength) \
     + ' bytes long, which is bigger than buffer size! Increase buffer length.')
     
    #Frame no.
    frameNum = struct.unpack_from('<l', data, dataIdx)[0] #THIS VALUE IS ALWAYS -1
    dataIdx += struct.calcsize('!l')
#     print frameNum
     
    #Number of markersets
    numMarkerSets = struct.unpack_from('<l', data, dataIdx)[0]
    dataIdx += struct.calcsize('!l')
#     print numMarkerSets
    
    #Marker sets
    markerSets = {}
     
    for _ in range(numMarkerSets):
        #Markerset name
        byteSize = data[dataIdx:].index('\0')
        setName = struct.unpack_from('<{0}s'.format(byteSize), data, dataIdx)
        dataIdx += byteSize + 1
#         print setName, byteSize
        
        #Num markers
        numMarkers = struct.unpack_from('<l', data, dataIdx)[0]
        dataIdx += struct.calcsize('!l')
#         print numMarkers
     
        markerSets[setName] = array(struct.unpack_from('<{0}f'.format(3*numMarkers), data, dataIdx)).reshape(numMarkers, 3)
        dataIdx += 3*numMarkers*struct.calcsize('!f')
#         print markerSets[setName]


    #Unidentified markers
    numUnMarkers = struct.unpack_from('<l', data, dataIdx)[0]
    dataIdx += struct.calcsize('!l')
#     print numUnMarkers
     
    #Unidentified marker position
    unMarker = array(struct.unpack_from('<{0}f'.format(3*numUnMarkers), data, dataIdx)).reshape(numUnMarkers, 3)
    dataIdx += 3*numUnMarkers*struct.calcsize('!f')
#     print unMarker

    #Num rigid bodies
    numRigidBodies = struct.unpack_from('<l', data, dataIdx)[0]
    dataIdx += struct.calcsize('!l')
#     print numRigidBodies
    
    rigidBodies = {}
     
    #Process rigid bodies
    for _ in range(numRigidBodies):
        #Rigid body ID
        rID = struct.unpack_from('<l', data, dataIdx)[0]
        dataIdx += struct.calcsize('!l')
        #print 'Rigid id:', rID
         
        #Rigid body state: [x y z], and [q1 q3 q2 q0]
        rSE3 = struct.unpack_from('<7f', data, dataIdx)
        dataIdx += struct.calcsize('!7f')
        rSE3 = rSE3[:3] + (rSE3[6], rSE3[3], rSE3[5], rSE3[4])
        #print 'Pos, ori:', rSE3
         
        #Number of rigid markers
        numRigidMarkers = struct.unpack_from('<l', data, dataIdx)[0]
        dataIdx += struct.calcsize('!l')
#         print numRigidMarkers
        
        rigidMarkers = {}
        
        #Rigid marker data
        rigidMarkers['coords'] = array(struct.unpack_from('<{0}f'.format(3*numRigidMarkers), data, dataIdx)).reshape(numRigidMarkers, 3)
        dataIdx += 3*numRigidMarkers*struct.calcsize('!f')
#         print rigidMarkers['coords']
         
        #Associated marker IDs
        rigidMarkers['ID'] = array(struct.unpack_from('<{0}l'.format(numRigidMarkers), data, dataIdx), dtype='int32')
        dataIdx += numRigidMarkers*struct.calcsize('!l')
#         print rigidMarkers['ID']
         
        #Associated marker byteSizes
        rigidMarkers['sizes'] = array(struct.unpack_from('<{0}f'.format(numRigidMarkers), data, dataIdx))
        dataIdx += numRigidMarkers*struct.calcsize('!l')
#         print rigidMarkers['sizes']
         
        #Mean marker error
        meanError = struct.unpack_from('<f', data, dataIdx)[0]
        dataIdx += struct.calcsize('!f')
#         print meanError
        
        rigidBodies[rID] = RigidBody(rID, rSE3, rigidMarkers, meanError)
    
    return markerSets, rigidBodies


if __name__ == '__main__':
    #TODO: test
    pass