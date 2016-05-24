#!/usr/bin/env python
'''
    Controller program for DrRobot X80Pro and ScoutPro robots. (ROS Node)
    Copyright (C) 2015-2016  Cristian Ioan Vasile <cvasile@bu.edu>
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
import threading as th

import rospy
from geometry_msgs.msg import Vector3

from robots import ScoutPro
from robots import X80Pro


control_value = None
lock = th.Lock()

def set_control(data):
    global control_value, lock
    with lock:
        if data:
            control_value = (data.x, data.y)

def get_control():
    global control_value, lock
    with lock:
        return control_value

'''Note: The -pi to pi convention is used for angles.'''

def executeController(robot, rid):
    
#    robot.start() # start robot updating thread
    
#    robot.EnableAllSensorSending() # enable all sensors
#    time.sleep(0.1)
    
    while not rospy.is_shutdown(): # condition
        control_val = get_control()
        if control_val is None:
            linearVelocity, angularVelocity = 0, 0
        else:
            linearVelocity, angularVelocity = control_val
#             linearVelocity = min(linearVelocity, robot.pulse2vel(900))
#             angularVelocity = max(min(angularVelocity, robot.pulse2vel(800)), robot.pulse2vel(-800))
        print 'Lin Velocity: ', linearVelocity, robot.vel2pulse(linearVelocity)
        print 'Ang Velocity: ', angularVelocity, robot.vel2pulse(angularVelocity)#, robot.pulse2vel(1500)
        print
        # 3. send commands to the robot
#        linearVelocity = 0 #TODO: remove after test
        robot.setVelocity(linearVelocity, angularVelocity)
        print
        time.sleep(0.05)
    
    # stop robot
    robot.setSpeed([0, 0])
    robot.stopUpdate() # stop updating internal state
#     robot.join() # join robot update thread with main thread

def main():
    parser = argparse.ArgumentParser(description='Send velocity control values to robot.')
    parser.add_argument('-r', '--robot', action='store', default='ScoutPro1',
                        type=str,
                        help='select the robot to use (default: ScoutPro1)',
                        choices=['X80Pro1', 'X80Pro2', 'ScoutPro1', 'ScoutPro2'])
    parser.add_argument('-i', '--id', action='store', default=1, type=int,
                        help='select the rigid body id for OptiTrack associated with the robot (default: 1)')
    parser.add_argument('-t', '--tcpip', action='store', nargs=2, type=str,
                       help='host and port number for the tcp/ip connection',
                       default=('localhost', '11311'))
    args = parser.parse_args()
    print 'Script arguments:', args.robot, args.tcpip, args.id
    
    rospy.init_node('DrRobot_commander', anonymous=True)
    rospy.Subscriber("GroundCommand{}".format(args.robot), Vector3, set_control)
    
    # create robot object
    if args.robot == 'X80Pro1':
        robot = X80Pro('DrRobot1', '192.168.0.202', 10001) 
    elif args.robot == 'X80Pro2':
        robot = X80Pro('DrRobot1', '192.168.0.201', 10001)
    elif args.robot == 'ScoutPro1':
        robot = ScoutPro('DrRobot1', '192.168.0.220', 10002)
    elif args.robot == 'ScoutPro2':
        robot = ScoutPro('DrRobot1', '192.168.0.218', 10002)
    else:
        raise ValueError('Unknown robot!')
     
    try:
        executeController(robot, args.id)
    finally:
        robot.setSpeed([0, 0])

if __name__ == '__main__':
    main()
