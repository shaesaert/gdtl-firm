#!/usr/bin/env python
license_text='''
    Module defines motion models.
    Copyright (C) 2016  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University

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

import itertools as it
import logging

import numpy as np
try:
    import rospy
    from geometry_msgs.msg import Vector3
except ImportError as ie:
#     logging.warning('Could not import rospy module: %s', ie)
    pass

from se2beliefspace import SE2BeliefState


class MotionModelMethod(object):
    '''Abstract class interface for defining a motion model.'''
    
    def __init__(self, stateDim=0, controlDim=0, noiseDim=0, isSimulation=True):
        self.stateDim = stateDim # dimension of the system state
        self.controlDim = controlDim # control vector dimension is specific to each motion model subclass
        self.noiseDim = noiseDim # noise vector dimension is specific to each motion model subclass
        self.zeroState = np.zeros((self.stateDim, )) # zero state
        self.zeroNoise = np.zeros((self.noiseDim, )) # zero noise
        self.zeroControl = np.zeros((self.controlDim, )) # zero control
        self.dt = 0.0 # timestep size, used to generate the next state by applying a control for this period of time
        self.isSimulation = isSimulation # flag for simulation mode
    
    def evolve(self, state, control, w=None):
        '''Propagates the system to the next state, given the current state,
        a control and a noise.
        '''
        raise NotImplementedError
    
    def generateOpenLoopControls(self, startState, endState):
        '''Generate open loop control that drives robot from start to end state.
        Returns an array of controls, each one on a line.
        '''
        raise NotImplementedError
    
    def generateOpenLoopControlsForPath(self, path):
        '''Generate open loop controls for a geometric path.
        Returns an array of controls, each one on a line.
        '''
        return np.vstack([self.generateOpenLoopControls(i, f)
                          for i, f in it.izip(path[:-1], path[1:])])
    
    def generateOpenLoopTrajectory(self, startState, endState, pairs=False):
        '''Generate open loop control and state trajectories that drives robot
        from start to end state.
        Returns the arrays of state and controls, each one on a line.
        '''
        # get the open loop controls for this edge
        openLoopControls = self.generateOpenLoopControls(startState, endState)
        # generate the intermediate states using the open loop controls
        aux_state = startState
        intermediates = [aux_state.copy()]
        for u in openLoopControls:
            aux_state = self.evolve(aux_state, u)
            intermediates.append(aux_state)
        if pairs:
            return zip(intermediates, openLoopControls)
        return intermediates, openLoopControls
    
    def generateNoise(self, state, control):
        '''Generate noise according to specified state and control input.'''
        return np.random.multivariate_normal(mean=self.zeroNoise,
                               cov=self.processNoiseCovariance(state, control))
    
    def getStateJacobian(self, state, control, w=None):
        '''Calculate the state transition Jacobian i.e. df/dx where f is the
        transition function and x is the state.
        '''
        raise NotImplementedError
    
    def getControlJacobian(self, state, control, w=None):
        '''Calculate the control transition Jacobian i.e. df/du where f is the
        transition function and u is the control.
        '''
        raise NotImplementedError
    
    def getNoiseJacobian(self, state, control, w=None):
        '''Calculate the noise transition Jacobian i.e. df/dw where f is the
        transition function and w is the noise.
        '''
        raise NotImplementedError
    
    def processNoiseCovariance(self, state, control):
        '''Calculate the process noise covariance.'''
        raise NotImplementedError
    
    def getZeroControl(self):
        '''Get zero control which produces no change in the robot state.'''
        return self.zeroControl
    
    def getZeroNoise(self):
        '''Get the zero noise.'''
        return self.zeroNoise
    
    def execute(self, u, blocking=True):
        '''Sends the control u to the robot.'''
        raise NotImplementedError
    
    def executionMode(self, isSimulation):
        '''Sets the execution mode and performs appropriate setups.'''
        raise NotImplemented


class OmnidirectionalMotionModel(MotionModelMethod):
    # Dimensions of state vector, control vector, motion noise vector
    stateDim, controlDim, motionNoiseDim = 2, 2, 2
    
    def __init__(self, robotModel, isSimulation=True):
        super(OmnidirectionalMotionModel, self).__init__(
            OmnidirectionalMotionModel.stateDim,
            OmnidirectionalMotionModel.controlDim,
            OmnidirectionalMotionModel.motionNoiseDim,
            isSimulation)
        self.setup(robotModel)
    
    def evolve(self, state, control, w=None):
        '''Propagate the system to the next state, given the current state,
        a control and a noise.
        '''
        if w is None:
            w = self.zeroNoise
        u = np.array(control)
       
        x = state.getConf()
        x = x + self.dt * u + self.sqdt * w
        nextState = state.copy()
        nextState.setConf(x)
        return nextState
    
    def generateOpenLoopControls(self, startState, endState):
        '''Generate open loop control that drives robot from start to end state.'''
        start = startState.getConf()
        end = endState.getConf()
        
        x_c = np.array([start[0], end[0]])
        y_c = np.array([start[1], end[1]])
        
        # connecting line slope
        th_p = np.arctan2(y_c[1]-y_c[0], x_c[1]-x_c[0])
        
        delta_disp = np.sqrt( (y_c[1]-y_c[0])**2 + (x_c[1]-x_c[0])** 2 )
        translation_steps = np.abs(delta_disp/(self.maxLinearVelocity * self.dt))
        ts = np.ceil(translation_steps)
        openLoopControls = np.repeat([[self.maxLinearVelocity * np.cos(th_p),
                                       self.maxLinearVelocity * np.sin(th_p)]],
                                     [ts], axis=0)
        # final control signal for translation
        ts_delta = translation_steps - np.floor(translation_steps)
        if ts_delta > 0:
            openLoopControls[ts-1, 0] *= ts_delta
            openLoopControls[ts-1, 1] *= ts_delta
        
        logging.debug('[Omni] Start conf: %s, End conf: %s, Distance: %s, Steps: %s',
                      start, end, delta_disp, translation_steps)
        
        return openLoopControls
    
    def getStateJacobian(self, state, control, w=None):
        '''Calculate the state transition Jacobian i.e. df/dx where f is the
        transition function and x is the state.
        '''
        return np.mat(np.eye(2))
    
    def getControlJacobian(self, state, control, w=None):
        '''Calculate the control transition Jacobian i.e. df/du where f is the
        transition function and u is the control.
        '''
        return self.dt * np.mat(np.eye(2))
    
    def getNoiseJacobian(self, state, control, w=None):
        '''Calculate the noise transition Jacobian i.e. df/dw where f is the
        transition function and w is the noise.
        '''
        return self.sqdt * np.mat(np.eye(2))
    
    def processNoiseCovariance(self, state, control):
        '''Calculate the process noise covariance.'''
        return self.Q
    
    def execute(self, u, blocking=True):
        '''Sends the control u to the robot.'''
        if self.isSimulation:
            return
        if not rospy.is_shutdown():
            self.msg.x, self.msg.y = u
            self.pub.publish(self.msg)
            if blocking:
                self.rate.sleep()
    
    def executionMode(self, isSimulation):
        '''Sets the execution mode and performs appropriate setups.'''
        if isSimulation:
            self.pub = None
            self.rate = None
            self.msg = None
        else:
            # create subscriber and message to read data in
            self.pub = rospy.Publisher(self.publisher_name,
                                       Vector3, queue_size=10)
            self.rate = rospy.Rate(self.control_freq) # Hz
            self.msg = Vector3(0, 0, 0)
        self.isSimulation = isSimulation
    
    def setup(self, robotModel):
        mm = robotModel["motion_model"]
        self.minLinearVelocity = mm["min_linear_velocity"]
        self.maxLinearVelocity = mm["max_linear_velocity"]
        self.dt = mm["dt"] # time step
        self.sqdt = np.sqrt(self.dt) # square root of time step
        # covariance of state additive noise
        self.Q = np.mat(mm['process_noise_covariance'])
        
        logging.debug("OmnidirectionalMotionModel: Q = \n{}".format(self.Q))
        logging.debug("OmnidirectionalMotionModel: min Linear Velocity (m/s)    = %f", self.minLinearVelocity)
        logging.debug("OmnidirectionalMotionModel: max Linear Velocity (m/s)    = %f", self.maxLinearVelocity)
        logging.debug("OmnidirectionalMotionModel: Timestep (seconds) = %f", self.dt)
        
        self.publisher_name = mm['ROS_control_publisher_name']
        self.control_freq = mm['ROS_control_send_rate']
        self.executionMode(self.isSimulation)


class UnicycleMotionModel(MotionModelMethod):
    # Dimensions of state vector, control vector, motion noise vector
    # These are constant and specific to the motion model. This also implies
    # the motion model will only work with configurations of a particular dimension
    stateDim, controlDim, motionNoiseDim = 3, 2, 3
     
    def __init__(self, robotModel, isSimulation=True):
        super(UnicycleMotionModel, self).__init__(UnicycleMotionModel.stateDim,
            UnicycleMotionModel.controlDim, UnicycleMotionModel.motionNoiseDim,
            isSimulation)
        self.setup(robotModel)
     
    def evolve(self, state, control, w=None):
        '''Propagate the system to the next state, given the current state,
        a control and a noise.
        '''
        if w is None:
            w = self.zeroNoise
        v, omega = control
        
        x = state.getConf()
        vp = self.dt * v
        omp = self.dt * omega
        x = x + np.array([vp * np.cos(x[2]), vp * np.sin(x[2]), omp]) \
              + self.sqdt * w
#         x[2] = normalizeAngle(x[2]) # normalize angle
        nextState = state.copy()
        nextState.setConf(x)
        return nextState
     
    def generateOpenLoopControls(self, startState, endState):
        '''Generate open loop control that drives robot from start to end state.'''
        start = startState.getConf() # turn into colvec (in radian)
        end = endState.getConf() # turn into colvec (in radian)
         
        x_c = np.array([start[0], end[0]])
        y_c = np.array([start[1], end[1]])
        theta_start = start[2]
#         theta_end = end[2]
         
        # connecting line slope
        th_p = np.arctan2(y_c[1]-y_c[0], x_c[1]-x_c[0])
         
        # turning angle at start and bringing the delta_th_p_start to the -PI to PI range
        delta_th_p_start = SE2BeliefState.normalizeAngle(th_p - theta_start)
        # turning angle at end and bringing the delta_th_p_start to the -PI to PI range
#         delta_th_p_end = normalizeAngle(theta_end - th_p)
        
        # count rotational steps
        rotation_steps_start = np.abs(delta_th_p_start/(self.maxAngularVelocity * self.dt))
        rsi = np.ceil(rotation_steps_start)
#         rotation_steps_end = np.abs(delta_th_p_end/(self.maxAngularVelocity * self.dt))
#         rsf = np.ceil(rotation_steps_end)
        
        # count translation steps
        delta_disp = np.sqrt( (y_c[1]-y_c[0])**2 + (x_c[1]-x_c[0])** 2 )
        translation_steps = np.abs(delta_disp/(self.maxLinearVelocity * self.dt))
        ts = np.ceil(translation_steps)
        
        openLoopControls = np.repeat([
                [0, self.maxAngularVelocity * np.sign(delta_th_p_start)],
                [self.maxLinearVelocity, 0]],
            [rsi, ts], axis=0)
         
        # final control signal for start rotation
        rsi_delta = rotation_steps_start - np.floor(rotation_steps_start)
        if rsi_delta > 0:
            openLoopControls[rsi-1, 1] *= rsi_delta
        # final control signal for translation
        ts_delta = translation_steps - np.floor(translation_steps)
        if ts_delta > 0:
            openLoopControls[rsi+ts-1, 0] *= ts_delta
#         # final control signal for end rotation
#         rsf_delta = rotation_steps_end - np.floor(rotation_steps_end)
#         if rsf_delta > 0:
#             openLoopControls[rsi+ts+rsf-1, 1] *= rsf_delta
        
#         logging.debug('[Unicycle] Start conf: %s, End conf: %s, Distance: %s, Steps: %s'
#                       + ' Initial rotation: %s Steps: %s, Final rotation: %s, Steps: %s',
#                       start, end, delta_disp, translation_steps, delta_th_p_start,
#                       rotation_steps_start, delta_th_p_start, rotation_steps_end)
        
        logging.debug('[Unicycle] Start conf: %s, End conf: %s, Distance: %s, Steps: %s'
                      + ' Rotation: %s Steps: %s',
                      start, end, delta_disp, translation_steps, delta_th_p_start,
                      rotation_steps_start)
        
        return openLoopControls
     
    def getStateJacobian(self, state, control, w=None):
        '''Calculate the state transition Jacobian i.e. df/dx where f is the
        transition function and x is the state.
        '''
        if w is None:
            w = self.zeroNoise
        vn = self.dt * control[0]
        theta = state.yaw
        return np.matrix([[1, 0, -vn * np.sin(theta)],
                          [0, 1,  vn * np.cos(theta)],
                          [0, 0,  1]])
     
    def getControlJacobian(self, state, control, w=None):
        '''Calculate the control transition Jacobian i.e. df/du where f is the
        transition function and u is the control.
        '''
        theta = state.yaw
        return self.dt * np.matrix([[np.cos(theta), 0],
                                    [np.sin(theta), 0],
                                    [0,             1]])
     
    def getNoiseJacobian(self, state, control, w=None):
        '''Calculate the noise transition Jacobian i.e. df/dw where f is the
        transition function and w is the noise.
        '''
        return self.sqdt * np.mat(np.eye(3))
     
    def processNoiseCovariance(self, state, control):
        '''Calculate the process noise covariance.'''
        return self.Q
     
    def execute(self, u, blocking=True):
        '''Sends the control u to the robot.'''
        if self.isSimulation:
            return
        if not rospy.is_shutdown():
            self.msg.x, self.msg.y = u
            self.pub.publish(self.msg)
            if blocking:
                self.rate.sleep()
     
    def executionMode(self, isSimulation):
        '''Sets the execution mode and performs appropriate setups.'''
        if isSimulation:
            self.pub = None
            self.rate = None
            self.msg = None
        else:
            # create subscriber and message to read data in
            self.pub = rospy.Publisher(self.publisher_name,
                                       Vector3, queue_size=10)
#             rospy.init_node('MotionModel', anonymous=True)
            self.rate = rospy.Rate(self.control_freq) # Hz
            self.msg = Vector3(0, 0, 0)
        self.isSimulation = isSimulation
     
    def setup(self, robotModel):
        mm = robotModel["motion_model"]
         
        # max translational velocity
        self.minLinearVelocity = mm["min_linear_velocity"]
        # min translational velocity
        self.maxLinearVelocity = mm["max_linear_velocity"]
        # max rotational velocity
        self.maxAngularVelocity = mm["max_angular_velocity"]
        self.dt = mm["dt"]
        self.sqdt = np.sqrt(self.dt)
        # covariance of state additive noise
        self.Q = np.array(mm['process_noise_covariance'])
         
        logging.debug("UnicycleMotionModel: Q = \n{}".format(self.Q))
        logging.debug("UnicycleMotionModel: min Linear Velocity (m/s)    = %f", self.minLinearVelocity)
        logging.debug("UnicycleMotionModel: max Linear Velocity (m/s)    = %f", self.maxLinearVelocity)
        logging.debug("UnicycleMotionModel: max Angular Velocity (rad/s) = %f", self.maxAngularVelocity)
        logging.debug("UnicycleMotionModel: Timestep (seconds) = %f", self.dt)
         
        self.publisher_name = mm['ROS_control_publisher_name']
        self.control_freq = mm['ROS_control_send_rate']
        self.executionMode(self.isSimulation)
