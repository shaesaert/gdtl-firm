'''
    #TODO:
    Copyright (C) 2013  Cristian Ioan Vasile <cvasile@bu.edu> and
                        Ethan Paul Bradlow <epbradlow@gmail.com>
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

import matplotlib.pyplot as plt
from ControllerDrRobot import Trajectory

def plotTrajectory(desired, measured):
    #set to false if not a repeating circuit
    closed_circuit = True
    
    xCoords, zCoords = measured.points.T
        
    if closed_circuit:
        desired.add(desired.points[1])
    xCoordsDesired, zCoordsDesired = desired.points.T

    #get the trajectory error distances and points
    distances = Trajectory.computeError(desired, measured)
            
    # Position of the robot in top subplot
    plt.subplot(211, aspect='equal')
    plt.plot(xCoords, zCoords)
    plt.plot(xCoordsDesired, zCoordsDesired)
        
    #Set the axis parameters
    xmin = min(xCoords)
    xmax = max(xCoords)
    zmin = min(zCoords)
    zmax = max(zCoords)
    plt.axis([xmin-0.1, xmax+0.1, zmin-0.1, zmax+0.1])
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Z Coordinate')
    
    
    #Deviation at each measured point in second subplot
    plt.subplot(212)
    x = range(len(distances)) 
    plt.step(x, distances)
    
    #Set the axis parameters
    plt.axis([0, len(x), 0, 0.5])
    
    plt.xlabel('Readings')
    plt.ylabel('Deviation (meters)')
   
    plt.show()

if __name__ == '__main__':
    import numpy as np
    plt.subplot(111, aspect='equal')
    x, y = np.loadtxt('track_data.txt', delimiter=' ', unpack=True)
    plt.plot(x, y, color='k')
    
    xd, yd = np.loadtxt('waypoints.txt', delimiter=' ', unpack=True)
    yd = -yd
    plt.plot(xd, yd, 'o-', color='b')
#     plt.plot(list(xd), list(yd), color='b')
    
    #Set the axis parameters
    xmin = min(x)
    xmax = max(x)
    ymin = min(y)
    ymax = max(y)
    plt.axis([xmin-0.1, xmax+0.1, ymin-0.1, ymax+0.1])
    plt.xlabel('X Coordinate')
    plt.ylabel('y Coordinate')
    
    plt.show()