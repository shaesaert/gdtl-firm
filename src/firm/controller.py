#!/usr/bin/env python
license_text='''
    Module implements the LQR/LQG feedback controllers.
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

import time
import logging
import itertools as it

import numpy as np
from scipy.linalg import solve_discrete_are as dare

from linearsystem import LinearSystem


class FeedbackController(object):
    '''Base class for a feedback controller without filtering.'''
    
    def __init__(self, motionModel, observationModel):
        self.motionModel = motionModel
        self.observationModel = observationModel
    
    def getCurrentLS(self, t):
        '''Retrieves the linear system for the current time step t.''' 
        raise NotImplementedError
    
    def getNextLS(self, t):
        '''Retrieves the linear system for the current time step t+1.'''
        raise NotImplementedError
    
    def isTerminated(self, state, t):
        '''Checks whether the controller has finished.'''
        raise NotImplementedError
    
    def generateFeedbackControl(self, state, t=None):
        '''Generates a feedback control based on the current state at time t.'''
        raise NotImplementedError


class SLQRController(FeedbackController):
    '''Class implements a (Stationary) Linear Quadratic Regulator (LQR) that
    drives a system towards a goal state.
    '''
    
    def __init__(self, motionModel, observationModel, target,
                 stateWeight, controlWeight):
        super(SLQRController, self).__init__(motionModel, observationModel)
        self.target = target
        self.Wx = stateWeight
        self.Wu = controlWeight
        # compute gain matrix
        u_zero = self.motionModel.getZeroControl()
        A = np.mat(self.motionModel.getStateJacobian(self.target, u_zero))
        B = np.mat(self.motionModel.getControlJacobian(self.target, u_zero))
        
        try:
            S = np.mat(dare(A, B, self.Wx, self.Wu))
        except:
            raise AssertionError("Dare Unsolvable: "
                               + "The given system state is not controllable.")
            return None
        # ensure that it is a real symmetric matrix
        assert np.all(S == S.T) and (np.all(np.isreal(S)))
        self.L = np.asarray((B.T * S * B + self.Wu).I * B.T * S * A)
        self.ls = LinearSystem(self.target, self.motionModel.getZeroControl(),
                                self.motionModel, self.observationModel)
    
    def getCurrentLS(self, t):
        return self.ls
    
    def getNextLS(self, t):
        return self.ls
    
    def isTerminated(self, state, t):
        '''Checks whether the controller has satisfied its termination condition
        for e.g. reached target state.
        '''
        return self.target.isReached(state)
    
    def generateFeedbackControl(self, state, t=None):
        return -self.L.dot(state.conf - self.target.conf)


class OffAxisSLQRController(FeedbackController):
    '''Class implements a (Stationary) Linear Quadratic Regulator (LQR) that
    drives a unicycle robot towards a goal state.
    '''
    
    def __init__(self, motionModel, observationModel, target,
                 stateWeight, controlWeight):
        super(OffAxisSLQRController, self).__init__(motionModel, observationModel)
        self.d = 0.1 #TODO: load from mission file
        self.target = target
        self.offaxis_target = self.getOffAxis(self.target)
        self.Wx = stateWeight[:2, :2]
        self.Wu = controlWeight[:2, :2]
        # compute gain matrix
        A = np.mat(np.eye(2))
        B = np.mat(self.motionModel.dt * np.eye(2))
        
        try:
            S = np.mat(dare(A, B, self.Wx, self.Wu))
        except:
            print A, B, self.Wx, self.Wu 
            raise AssertionError("Dare Unsolvable: "
                               + "The given system state is not controllable.")
            return None
        # ensure that it is a real symmetric matrix
        assert np.all(S == S.T) and (np.all(np.isreal(S)))
        self.L = np.asarray((B.T * S * B + self.Wu).I * B.T * S * A)
        self.ls = LinearSystem(self.target, self.motionModel.getZeroControl(),
                                self.motionModel, self.observationModel)
        
#         print 'Gain matrix:', self.L
    
    def getOffAxis(self, state):
        return np.array([state.x + self.d * np.cos(state.yaw),
                         state.y + self.d * np.sin(state.yaw)])
    
    def getCurrentLS(self, t):
        return self.ls
    
    def getNextLS(self, t):
        return self.ls
    
    def isTerminated(self, state, t):
        '''Checks whether the controller has satisfied its termination condition
        for e.g. reached target state.
        '''
        return self.target.isReached(state)
    
    def generateFeedbackControl(self, state, t=None):
        offaxis_state = self.getOffAxis(state)
        offaxis_control = -self.L.dot(offaxis_state - self.offaxis_target)
        cs, ss = np.cos(state.yaw), np.sin(state.yaw)
        T = np.array([[cs, ss], [-ss/self.d, cs/self.d]])
#         print 'target:', self.target.conf, 'offaxis-target:', self.offaxis_target, 'state:', state.conf, 'offaxis-state:', offaxis_state
#         print 'offaxis-control:', offaxis_control, 'offaxis-diff:', offaxis_state - self.offaxis_target,
#         print 'L:'
#         print self.L
#         print 'T:'
#         print T
#         print 'control:', T.dot(offaxis_control)
#         print
#         if t == 10:
#             assert False
        return T.dot(offaxis_control)


class SwitchingController(FeedbackController):
    '''Class implements a switching controller that drives a system towards a
    goal state.
    '''
    
    def __init__(self, motionModel, observationModel, target,
                 stateWeight, controlWeight):
        super(SwitchingController, self).__init__(motionModel, observationModel)
        self.target = target
#         self.Wx = stateWeight
#         self.Wu = controlWeight
#         # compute gain matrix
#         u_zero = self.motionModel.getZeroControl()
#         A = np.mat(self.motionModel.getStateJacobian(self.target, u_zero))
#         B = np.mat(self.motionModel.getControlJacobian(self.target, u_zero))
#         
#         try:
#             S = np.mat(dare(A, B, self.Wx, self.Wu))
#         except:
#             raise AssertionError("Dare Unsolvable: "
#                                + "The given system state is not controllable.")
#             return None
#         # ensure that it is a real symmetric matrix
#         assert np.all(S == S.T) and (np.all(np.isreal(S)))
#         self.L = np.asarray((B.T * S * B + self.Wu).I * B.T * S * A)
        self.a_epsilon = np.deg2rad(15) # TODO: read from mission file
        self.d_epsilon = 0.08
        self.kd_dist = 1.8 #200
        self.kd_heading = 1.6 #12.0
        self.max_linear_speed = 0.2
        self.min_linear_speed = 0.08
        self.max_angular_speed = 0.5
        self.ls = LinearSystem(self.target, self.motionModel.getZeroControl(),
                                self.motionModel, self.observationModel)
    
    def getCurrentLS(self, t):
        return self.ls
    
    def getNextLS(self, t):
        return self.ls
    
    def isTerminated(self, state, t):
        '''Checks whether the controller has satisfied its termination condition
        for e.g. reached target state.
        '''
        return self.target.isReached(state)
    
    def generateFeedbackControl(self, state, t=None):
        if t == 200:
            assert False
        error_dist = self.target.distanceTo(state)
        if self.target.isReached(state):
            return np.zeros((2,))
        
        xdiff, ydiff = self.target.x - state.x, self.target.y - state.y
        los_angle = np.arctan2(ydiff, xdiff)
        error_heading = los_angle - state.yaw # TODO: normlizeAngle()
        if error_heading > np.pi:
            error_heading = -(2*np.pi - error_heading)
        elif error_heading < -np.pi:
            error_heading = 2*np.pi + error_heading
        
        if abs(error_heading) < self.a_epsilon:
#             logging.debug('Full motion: error distance=%f, heading=%f, line-of-sight=%f',
#                           error_dist, error_heading, los_angle)
#             print 'Full motion: error distance=%f, heading=%f, line-of-sight=%f' % (error_dist, error_heading, los_angle)
            linear_velocity = self.kd_dist * error_dist
            linear_velocity = max(min(linear_velocity, self.max_linear_speed),
                                  self.min_linear_speed)
            angular_velocity = self.kd_heading * error_heading
            angular_velocity = max(min(angular_velocity,
                                       self.max_angular_speed),
                                   -self.max_angular_speed)
        else:
#             logging.debug('Rotate on spot: error distance=%f, heading=%f, line-of-sight=%f',
#                           error_dist, error_heading, los_angle)
            print 'Rotate on spot: error distance=%f, heading=%f, line-of-sight=%f state.yaw=%f' % (error_dist, error_heading, los_angle, state.yaw)
            linear_velocity = 0.0 # use constant linear velocity
            angular_velocity = self.kd_heading * error_heading
            angular_velocity = max(min(angular_velocity,
                                       self.max_angular_speed),
                                   -self.max_angular_speed)
            
        return np.array([linear_velocity, angular_velocity])


class TrackingLQRController(FeedbackController):
    '''Class implements a tracking Linear Quadratic Regulator (LQR) that drives
    a system along a nominal state and control trajectory.
    '''
    
    def __init__(self, motionModel, observationModel, transition=None, lss=None,
                 stateWeight=None, controlWeight=None):
        super(TrackingLQRController, self).__init__(motionModel, observationModel)
        
        if transition is None and lss is None:
            raise ValueError('Need either a transition or a nominal trajectory'
                             + '(i.e., sequence of linear systems)!')
        elif lss is None:
            initial, target = transition
#             print 'transition:', initial.conf, '->', target.conf#             print 'transition:', initial.conf, '->', target.conf
            nom_traj = motionModel.generateOpenLoopTrajectory(initial, target,
                                                              pairs=True)
            lss = [LinearSystem(x, u, motionModel, observationModel)
                                                        for x, u in nom_traj]
        self.lss = lss
        self.Wx = stateWeight
        self.Wu = controlWeight
        # compute control gain sequence
        dx = self.motionModel.stateDim
        du = self.motionModel.controlDim
        self.N = len(self.lss)
        S = np.empty((self.N+1, dx, dx), dtype=np.float)
        self.L = np.empty((self.N, du, dx), dtype=np.float)
        S[self.N,:, :] = self.Wx
#         self.L[self.N,:, :] = 0
        for t in it.count(self.N-1, -1):
            ls = self.lss[t]
            A, B, S_next = map(np.mat, [ls.getA(), ls.getB(), S[t+1, :, :]])
            L = (B.T * S_next * B + self.Wu).I * B.T * S_next * A
            self.L[t, :, :] = L
            S[t, :, :] = self.Wx + A.T * S_next * (A - B * L)
            if t == 0:
                break
    
    def getCurrentLS(self, t):
        assert 0 <= t < self.N
        return self.lss[t]
    
    def getNextLS(self, t):
        assert 0 <= t < self.N
        return self.lss[t]
    
    def isTerminated(self, state, t):
        assert 0 <= t <= self.N
        return t == self.N
    
    def generateFeedbackControl(self, state, t):
        assert 0 <= t <= self.N
        x_p, u_p = self.lss[t].x, self.lss[t].u
        print 'desired:', x_p.conf, u_p
        print 'comp:', state.conf, u_p - self.L[t, :, :].dot(state.conf - x_p.conf)
        print 'error:', state.conf - x_p.conf, self.L[t, :, :].dot(state.conf - x_p.conf)
        print
        return u_p - self.L[t, :, :].dot(state.conf - x_p.conf)


class NullController(FeedbackController):
    '''Class implements a placeholder controller that does nothing.'''
    
    def __init__(self, motionModel, observationModel, transition=None, lss=None,
                 stateWeight=None, controlWeight=None):
        super(NullController, self).__init__(motionModel, observationModel)
        
        if transition is None and lss is None:
            raise ValueError('Need either a transition or a nominal trajectory'
                             + '(i.e., sequence of linear systems)!')
        elif lss is None:
            _, target = transition
            lss = [LinearSystem(target, motionModel.zeroControl,
                                motionModel, observationModel)]
        self.ls = lss[0]
        self.Wx = stateWeight
        self.Wu = controlWeight
   
    def getCurrentLS(self, t):
        assert 0 <= t < self.N
        return self.lss[t]
    
    def getNextLS(self, t):
        return self.ls
    
    def isTerminated(self, state, t):
        return True
    
    def generateFeedbackControl(self, state, t):
        return self.ls.u


class Controller(object):
    '''Base class for Controller. A controller's task is to use the filter to
    estimate the belief robot's state and generate control commands using the
    separated controller. For example by fusing an LQR and Kalman Filter we
    generate an LQG controller.'''
    
    def __init__(self, motionModel, observationModel, controller, bayesianFilter,
                 withNoise=True, predObs=False):
        self.motionModel = motionModel
        self.observationModel = observationModel
        # feedback controller used to generate the controls for the robot
        self.feedbackController = controller
        # Bayesian filter used to estimate the robot belief
        self.filter = bayesianFilter
        self.withNoise = withNoise
        self.predObs = predObs
    
    def execute(self, startState, process=None):
        '''Execute the controller i.e. take the system from start to end state
        of edge.
        '''
        updatedState = startState
        for k in it.count():
            logging.debug('[Controller] Step: %d', k)
            t0 = time.time()
            if process:
                process.send(updatedState)
            if self.feedbackController.isTerminated(updatedState, k):
                break
            updatedState = self.evolve(updatedState, k)
            logging.debug('[Controller] execute step %d took: %f msec', k, (time.time() - t0)*1000)
        if process:
            return updatedState, process.send(updatedState)
        return updatedState
    
    def evolve(self, state, t, nocontrol=False): #FIXME: HACK
        '''Evolve the controller over a single time step, i.e., get control,
        apply control, get observation, apply filter.
        '''
        # get control
        if nocontrol:
            control = self.motionModel.zeroControl
        else:
            control = self.feedbackController.generateFeedbackControl(state, t)
        # apply control
        if self.withNoise:
            w = self.motionModel.generateNoise(state, control)
#             logging.debug('[Controller] evolve: process noise: %s', w)
        else:
            w = None
        nextState = self.motionModel.evolve(state, control, w)
#         logging.debug('[Controller] evolve: control=%s', control)
#         print '[controller] evolve: control=', control
        self.motionModel.execute(control)
        # get output estimate from observation model
        if self.predObs: # or True: #TODO: this is not good, need to be able to control if observation has noise or not vs if it is from simulation or real robot
            z = self.observationModel.getObservationPrediction(nextState)
        else:
            z = self.observationModel.getObservation(nextState)
#         logging.debug('[Controller] evolve: state: %s next state: %s observation: %s', state.conf, nextState.conf, z)
#         logging.debug('[Controller] evolve: target: %s', self.feedbackController.target.conf)
        # apply filter and get state estimate
        currentLS = self.feedbackController.getCurrentLS(t)
        nextLS = self.feedbackController.getNextLS(t)
        return self.filter.evolve(state, control, z, currentLS, nextLS)
