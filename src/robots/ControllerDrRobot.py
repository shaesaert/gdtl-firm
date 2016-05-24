'''
    Controller program for DrRobot X80Pro and ScoutPro robot.
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

import time
import argparse

from numpy import array, dot, vstack, zeros, arctan2, rad2deg
from numpy import loadtxt, savetxt
from numpy.linalg import norm

from robots import ScoutPro
from robots import X80Pro
from optitrack import OptiTrackInterface
from robots.GUIHandler import getWaypointsFromGUI


'''Note: The -pi to pi convention is used for angles.'''


class Trajectory(object):
    '''An class storing trajectories as NumPy arrays.'''
    
    def __init__(self, points=None, dimension=2):
        '''
        '''
        if points:
            self.points = array(points)
            self.dimension = self.points.shape[1]
        else:
            self.points = zeros([0, dimension])
            self.dimension = dimension
        self.currentTarget = None
    
    def add(self, point):
        '''Adds a point to the end of the trajectory.'''
        self.points = vstack([self.points, point])
    
    def setup(self, epsilon=0.1):
        '''TODO:'''
        no_points = self.points.shape[0]
        if no_points == 0:
            raise UserWarning('There are no points in the trajectory!')
        elif no_points == 1:
            self.targets = iter(self.points)
        else:
            self.targets = iter(self.points[1:])
        self.currentTarget = next(self.targets)
        self.epsilon = epsilon
    
    def getTarget(self, point):
        '''Returns the current target.'''
        distance = norm(self.currentTarget - point)
        print "Distance: ", distance
        if distance < self.epsilon: # if target was reached, switch to next one
            print "WITHIN EPSILON RANGE"
            try:
                self.currentTarget = next(self.targets)
            except StopIteration: # last target reached
                return None, 0, 0
            distance = norm(self.currentTarget - point)
        # compute heading of line of sight vector
        vector = self.currentTarget - point
        losAngle = arctan2(vector[1], vector[0])
        losAngle = rad2deg(losAngle)

        print 'self.currentTarget: ', self.currentTarget        
        return self.currentTarget, distance, losAngle
    
    @classmethod
    def computeError(cls, desired, measured, wp_threshold=0.11):
        '''
        Computed the error between a desired and measured trajectory.
        # TODO: add detailed description.
        '''
        targets = iter(desired.points)
        start = next(targets) # start point of the current segment
        target = next(targets) # target end-point of the current segment
        u = (target - start)/norm(target - start) # unit direction vector
        
        error = []
        for point in measured.points: # loop over all measured points
            distance2target = norm(target - point)
            if distance2target < wp_threshold: # if target was reached
                # update start and target end-points for the next segments
                start = target
                try:
                    target = next(targets)
                except StopIteration:
                    targets = iter(desired.points[1:]) # FIXME: does not work when initial point is not from OptiTrack
                    target = next(targets) # last target

                # recompute the unit direction vector
                u = (target - start)/norm(target - start)
            
            # compute the distance from the point to the segment
            w = point - start
            error.append(norm(w - dot(w, u)*u))
        
        return error

#The controller for the orientation and distance
def pidController(kp, ki, kd):
    ierror = 0
    prev_error = 0
    error = 0
    
    while True:
        ierror += error
        derror = error - prev_error
        prev_error = error
        u = kp*error + ki*ierror + kd*derror
        print "U: ", u
        error = (yield u)


def executeController(robot, waypoints=None, laps=2):
#     robot = X80Pro('DrRobot1', '192.168.0.202', 10001) # create robot object
#     robot = ScoutPro('DrRobot1', '192.168.0.220', 10002) # create robot object
#     opti = OptiTrackInterface()

    desired = Trajectory()
    measured = Trajectory()
    
    # add points to desired trajectory
    opti = OptiTrackInterface()
    body = opti.trackInfo()[1] # get data for rigid body w/ id 1
    opti._close()
    x, z, theta = body.x, -body.z, body.yaw
    print x, z, theta
    # initial position in desired points list
    desired.add([x, z])
    
    if len(waypoints) > 0:
        # add waypoints to the desired trajectory
        for x_des, z_des in waypoints:
            desired.add([x_des, z_des])
    else:
        desired.add([0, 0])
    desired.setup()
    
    print 'Trajectory:', desired.points

#    robot.start() # start robot updating thread
    
    runtime = 240 # seconds
    t0 = time.time() # initial time
    
#    robot.EnableAllSensorSending() # enable all sensors
#    time.sleep(0.1)

    pidDist = pidController(3200, 0, 0)
    pidDist.next() # start the pid controller
    pidHeading = pidController(0.1, 0, 0)
    pidHeading.next() # start the pid controller
    
    loop = 0
    while time.time() - t0 < runtime: # condition
        # 1. read sensors -- OptiTrack
        opti = OptiTrackInterface()
        body = opti.trackInfo()[1] # get data for rigid body w/ id 1
        opti._close()
        x, z, theta = body.x, -body.z, body.yaw
#         print body.yaw, body.pitch, body.roll

        # append position to measured trajectory
        measured.add([x, z])
        
        # 2. compute control values -- speeds for the two motors
        target, dist, losAngle = desired.getTarget([x, z])
        
        if target is None:
            #print 'Current position1:', x, z, theta
            #Shows exactly when one cycle has been completed
            print 'Reached final target', measured.points.shape[0]
            #Loops through again
            desired.setup()
            loop += 1
            if loop == laps:
                break
        
        # compute error values for distance and heading
        errorDist = dist
            
        errorHeading = losAngle - theta
        if errorHeading > 180:
            errorHeading = -(360 - errorHeading)
        elif errorHeading < -180:
            errorHeading = 360 + errorHeading
        
        print 'Target:', target
        print 'Current position:', x, z, theta
        print 'Distance error:', errorDist
        print 'losAngle: ', losAngle
        print 'theta: ', theta
        print 'Heading error:', errorHeading
        print
             
        # compute linear and angular velocities
        linearVelocity = pidDist.send(errorDist) # use PID for linear velocity
#         linearVelocity = min(linearVelocity, robot.pulse2vel(1000))
#         linearVelocity = robot.pulse2vel(800) # use constant linear velocity
        linearVelocity = robot.pulse2vel(0) # use constant linear velocity
        print 'Lin Velocity: ', robot.vel2pulse(linearVelocity)
        
        angularVelocity = pidHeading.send(errorHeading)
        angularVelocity = max(min(angularVelocity, robot.pulse2vel(800)), -robot.pulse2vel(800))
        print 'Ang Velocity: ', robot.vel2pulse(angularVelocity)
        
        # 3. send commands to the robot
        robot.setVelocity(linearVelocity, angularVelocity)
        time.sleep(0.1)
        
    else:
        print 'Time elapsed!'
        # stop robot
        robot.setSpeed([0, 0])

        robot.stopUpdate() # stop updating internal state
#         robot.join() # join robot update thread with main thread
    
    # stop robot
    robot.setSpeed([0, 0])

    robot.stopUpdate() # stop updating internal state
#     robot.join() # join robot update thread with main thread


    return desired, measured

def testOptitrack():
    opti = OptiTrackInterface()
    body = opti.trackInfo()[1] # get data for rigid body w/ id 1
    opti._close()
    x, z, theta = body.x, -body.z, body.yaw
    
    print '(X, Z, theta) = ', (x, z, theta)

def testMovement():
    robot = ScoutPro('DrRobot1', '192.168.0.220', 10002) # create robot object
    robot.setVelocity(0, 200)
    time.sleep(2)
    robot.setVelocity(0, 0)


def parseArgs():
#     zCoords, xCoords = loadtxt('wayPoints.txt', unpack=True)
#     xCoords, zCoords = loadtxt('waypoints.txt', unpack=True)
#     xCoords, zCoords = ([1.439], [0.981])

    #Assign the desired coordinates to the ones given by the GUI
#     HOST, PORT = "192.168.0.102", 9000
#     xCoords, zCoords = getWaypointsFromGUI(HOST, PORT)
    
#     print 'X coords:', xCoords
#     print 'Z coords:', zCoords

#     testOptitrack()
#     testMovement()

    parser = argparse.ArgumentParser(description='Trajectory following.')
    parser.add_argument('-r', '--robot', action='store', default='ScoutPro1',
                        type=str,
                        help='select the robot to use (default: ScoutPro1)',
                        choices=['X80Pro1', 'X80Pro2', 'ScoutPro1', 'ScoutPro2'])
    
    parser.add_argument('-l', '--laps', action='store', default=1, type=int,
                        help='the number of laps to perform (default: 1)')
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-f', '--file', action='store',
                       type=argparse.FileType('r'),
                       help='a file with waypoints on each line')
    group.add_argument('-w', '--waypoints', action='store', metavar='N',
                       type=float, nargs='+',
                       help='pairs of coordinates for the waypoints')
    group.add_argument('-t', '--tcpip', action='store', nargs=2, type=str,
                       help='host and port number for the tcp/ip connection')
    
    parser.add_argument('-p', '--plot', action='store_true',
                        help='flag for plotting the trajectory and error graphs')
    
    parser.add_argument('-s', '--save-trajectory', action='store',
                        type=argparse.FileType('w'),
                        help='output file to save measured trajectory',
                        dest='save')
    
    args = parser.parse_args()
    
    if args.file:
        waypoints = loadtxt(args.file)
        assert len(waypoints.shape) == 2
        assert waypoints.shape[1] == 2
    elif args.tcpip:
        waypoints = getWaypointsFromGUI(args.tcpip[0], int(args.tcpip[1]))
    elif args.waypoints:
        waypoints = array(args.waypoints).reshape(-1, 2)
    else:
        assert False, 'This code should be unreachable!'
    
    # transform to used coordinate system
    waypoints[:,1] = -waypoints[:,1]
    
    print args.robot
    print waypoints
    print args.laps
    print args.plot
    print args.save

    return args.robot, waypoints, args.laps, args.plot, args.save


if __name__ == '__main__':
    robot_name, waypoints, laps, plot_traj, save_traj = parseArgs()
    
    # create robot object
    if robot_name == 'X80Pro1':
        robot = X80Pro('DrRobot1', '192.168.0.202', 10001) 
    elif robot_name == 'X80Pro2':
        robot = X80Pro('DrRobot1', '192.168.0.201', 10001)
    elif robot_name == 'ScoutPro1':
        robot = ScoutPro('DrRobot1', '192.168.0.220', 10002)
    elif robot_name == 'ScoutPro2':
        robot = ScoutPro('DrRobot1', '192.168.0.218', 10002)
    else:
        raise ValueError('Unknown robot!')
     
    try:
        desired, measured = executeController(robot, waypoints, laps)
    finally:
        robot.setSpeed([0, 0])
     
#     if plot_traj:
#         from robots.plotTrajectory import plotTrajectory
#         plotTrajectory(desired, measured)
    if save_traj:
        # save measured points
        savetxt(save_traj, measured.points, newline='\n')
