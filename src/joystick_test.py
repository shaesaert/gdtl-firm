#!/usr/bin/env python
'''
Created on Feb 23, 2016

@author: cristi
'''

import pygame
pygame.joystick.init()
pygame.init()
import rospy
from geometry_msgs.msg import Vector3


def joystick():
    assert pygame.joystick.get_count() > 0, pygame.joystick.get_count()
    
    # initialize ros node
    rospy.init_node("JoystickControl", anonymous=True)
    
    # create subscriber and message to read data in
    pub = rospy.Publisher("GroundCommandX80Pro2", Vector3, queue_size=10)
#             rospy.init_node('MotionModel', anonymous=True)
    rate = rospy.Rate(4) # Hz
    msg = Vector3(0, 0, 0)
    
    # create joystick object from first device
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    axes = [ 0.0 ] * joystick.get_numaxes()
    buttons = [ False ] * joystick.get_numbuttons()
    
    keep_alive = True
    while keep_alive:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYAXISMOTION:
                e = event.dict
                axes[e['axis']] = e['value']
            elif event.type == pygame.JOYBUTTONUP:
                e = event.dict
                buttons[e['button']] = False
            elif event.type == pygame.JOYBUTTONDOWN:
                e = event.dict
                buttons[e['button']] = True
            
            print 'Axes:', axes, 'Buttons:', buttons
        
        if buttons[0]:
            # form message and send it to robot
#             msg.x, msg.y = (-axes[1]/1.5, -axes[0]/1.0)
            msg.x, msg.y = (-axes[1]/3.0, -axes[0]/2.0)
        else:
            msg.x, msg.y = 0, 0
        
        if buttons[1]:
            keep_alive = False
        
        pub.publish(msg)
        rate.sleep() # sleep
    
    msg.x, msg.y = 0, 0
    pub.publish(msg)
    pygame.quit()

if __name__ == '__main__':
    joystick()