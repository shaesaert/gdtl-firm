#!/usr/bin/env python
license_text='''
    Module defines observation models.
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
import logging

import numpy as np
from threading import Lock

try:
    import rospy
    from geometry_msgs.msg import Twist, PoseStamped
except ImportError as ie:
#     logging.warning('Could not import rospy module: %s', ie)
    pass


class ObservationModelMethod:
    '''Base class for observation models. An observation model contains a
    description of the sensor. It generates observations and associated
    observation Jacobians/noises.
    '''
    
    def __init__(self, nDim=0, isSimulation=True):
        '''Constructor.'''
        self.noiseDim = nDim # noise vector dimension
        self.zeroNoise = np.zeros((nDim,)) # zero noise vector
        self.isSimulation = isSimulation # flag for simulation mode
    
    def getObservation(self, state):
        '''z = h(x,v). Get the observation for a given configuration.
        \param The state based on which the observation should be generated.
        \param if true, observation corrupted by noise from a given
        distribution, otherwise noise free observation.
        '''
        raise NotImplementedError
    
    def getObservationPrediction(self, state, v=None):
        '''Get the predicted observation based on predicted state.'''
        raise NotImplementedError
    
    def getObservationJacobian(self, state, v=None):
        '''Calculate the observation Jacobian i.e. Jx = dh/dx, where h is
        the observation model and x is the state.
        '''
        raise NotImplementedError
    
    def getNoiseJacobian(self, state, v=None):
        '''Calculates the observation noise Jacobian i.e. Jv = dh/dv, where h is
        the observation model and v is the noise.
        '''
        raise NotImplementedError
    
    def getObservationNoiseCovariance(self, state):
        '''Calculates the observation noise covariance.'''
        raise NotImplementedError
    
    def isStateObservable(self, state):
        '''Checks if a state is observable.'''
        raise NotImplementedError
    
    def getZeroNoise(self):
        '''Returns the zero observation noise.'''
        return self.zeroNoise
    
    def executionMode(self, isSimulation):
        '''Sets the execution mode and performs appropriate setups.'''
        raise NotImplemented


class CameraLocalization(ObservationModelMethod):
    '''z = h(x,v) get the observation for a given configuration, corrupted by
    noise from a given distribution
    '''
    
    def __init__(self, robotModel, env, nDim=3, isSimulation=True):
        ObservationModelMethod.__init__(self, nDim, isSimulation)
        om = robotModel['observation_model']
        
        self.executionMode(self.isSimulation)
        
        self.C = np.array(om['observation_jacobian'])
        self.M = np.array(om['noise_jacobian'])
        self.Rs = np.array(om['observation_noise_covariances'])
        
#         # cover map parameters
#         self.cover_map = np.loadtxt(om['cover_map']['filename'], delimiter=',')
# #         self.cover_min_cam = int(om['cover_map']['minimum_number_of_cameras'])
#         x_range, y_range = np.array(env['boundary']['limits']).T
#         self.cover_inc_x = self.cover_map.shape[1] / np.diff(x_range)
#         self.cover_inc_y = self.cover_map.shape[0] / np.diff(y_range)
#         self.cover_dim = np.array(self.cover_map.shape)-1
#         assert np.isclose(self.cover_inc_x, self.cover_inc_y), (self.cover_inc_x, self.cover_inc_y) 
    
    def executionMode(self, isSimulation):
        '''Sets the execution mode and performs appropriate setups.'''
        if isSimulation:
            self.__lock = Lock()
            self.__z = None
        else:
            # create subscriber and message to read data in
#             rospy.init_node('ObservationModel', anonymous=True)
#             rospy.Subscriber("pose", Twist, lambda data: self.__save(data))
            rospy.Subscriber("robot_pose_estimation", Twist, lambda data: self.__save(data))
            self.__lock = Lock()
            self.__z = None
        self.isSimulation = isSimulation
    
    def __save(self, data):
        self.__lock.acquire()
        self.__z = (data.linear.x, data.linear.y, data.angular.z)
        self.__lock.release()
        logging.debug('[Camera] received: %s', self.__z)
#         print '[Camera] received:', self.__z
    
    def getObservationPrediction(self, state, v=None):
        '''Get the predicted observation based on predicted state.'''
        if v is None:
            return self.C.dot(state.conf)
        return self.C.dot(state.conf) + v
    
    def getObservation(self, state):
        '''Return an observation at the given state.'''
        z = np.zeros((self.noiseDim,))
        if self.isSimulation:
            # generate observation
            x = state.getConf()
            v = np.random.multivariate_normal(self.zeroNoise,
                                      self.getObservationNoiseCovariance(state))
            c = self.getObservationNoiseCovariance(state)
            logging.debug('[ObservationModel] getObservation() noise v= %s cov: %s',
                          v, np.diag(c))
            z[:] = self.C.dot(x) + v
        else:
            # get data from ROS subscriber and copy it into z
            with self.__lock:
                z[:] = self.__z
            logging.debug('[ObservationModel] getObservation(): %s', z)
        return z
    
    def getObservationJacobian(self, state, v=None):
        return self.C
    
    def getNoiseJacobian(self, state, v=None):
        return self.M
    
    def getObservationNoiseCovariance(self, state):
#         coord =  np.array(np.floor([state.y*self.cover_inc_y,
#                                     state.x*self.cover_inc_x]),
#                           dtype=np.intp).flatten()
#         if np.any(coord > self.cover_dim) or np.any(coord < 0):
#             return self.Rs[0]
#         r, c = coord
#         cover_no_cam = np.asscalar(self.cover_map[r, c])
#         logging.debug('[Camera] Obs Noise Cov: conf: %s, camera number: %s, diag(cov): %s',
#                        state.conf, cover_no_cam, np.diag(self.Rs[cover_no_cam]))
#         return self.Rs[cover_no_cam]
        return self.Rs[1]
    
    def isStateObservable(self, state):
        return True # assume that all states are observable


class OptitrackLocalization(ObservationModelMethod):
    '''z = h(x,v) get the observation for a given configuration, corrupted by
    noise from a given distribution
    '''
    
    def __init__(self, robotModel, env, nDim=3, isSimulation=True):
        ObservationModelMethod.__init__(self, nDim, isSimulation)
        if not self.isSimulation:
            self.setupROS()
        
        self.C = np.eye(3)
        self.M = np.zeros((3, 3))
        self.R = np.zeros((3, 3))
    
    def setupROS(self):
        # create subscriber and message to read data in
#         rospy.init_node('OptitrackClient', anonymous=True)
        rospy.Subscriber("truepose", Twist, lambda data: self.__save(data))
        self.__lock = Lock()
        self.__z = None
    
    def __save(self, data):
        self.__lock.acquire()
        self.__z = (data.linear.x, data.linear.y, data.angular.z)
        self.__lock.release()
    
    def getObservationPrediction(self, state, v=None):
        '''Get the predicted observation based on predicted state.'''
        return state.copy()
    
    def getObservation(self, state=None):
        '''Return an observation at the given state.'''
        z = np.zeros((self.noiseDim,))
        if self.isSimulation:
            # generate observation
            z[:] = state.getConf()
        else:
            # get data from ROS subscriber and copy it into z
            with self.__lock:
                z[:] = self.__z
        return z
    
    def getObservationJacobian(self, state, v=None):
        return self.C
    
    def getNoiseJacobian(self, state, v=None):
        return self.M
    
    def getObservationNoiseCovariance(self, state):
        return self.R
    
    def isStateObservable(self, state):
        return True # assume that all states are observable


class QuadCamPose(ObservationModelMethod):
    '''z = h(x,v) get the observation for a given configuration, corrupted by
    noise from a given distribution
    '''
    
    def __init__(self, robotModel, env, nDim=7, isSimulation=True):
        ObservationModelMethod.__init__(self, nDim, isSimulation)
        if not self.isSimulation:
            self.setupROS()
        
        self.C = np.eye(3)
        self.M = np.zeros((3, 3))
        self.R = np.zeros((3, 3))
    
    def setupROS(self):
        # create subscriber and message to read data in
#         rospy.init_node('OptitrackClient', anonymous=True)
        rospy.Subscriber("/Robot_2/pose", PoseStamped, lambda data: self.__save(data))
        self.__lock = Lock()
        self.__z = None
    
    def __save(self, data):
        self.__lock.acquire()
        self.__z = (data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.position.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        self.__lock.release()
    
    def getObservationPrediction(self, state, v=None):
        '''Get the predicted observation based on predicted state.'''
        return state.copy()
    
    def getObservation(self, state=None):
        '''Return an observation at the given state.'''
        z = np.zeros((self.noiseDim,))
        if self.isSimulation:
            # generate observation
            z[:] = state.getConf()
        else:
            # get data from ROS subscriber and copy it into z
            with self.__lock:
                z[:] = self.__z
        return z
    
    def getObservationJacobian(self, state, v=None):
        return self.C
    
    def getNoiseJacobian(self, state, v=None):
        return self.M
    
    def getObservationNoiseCovariance(self, state):
        return self.R
    
    def isStateObservable(self, state):
        return True # assume that all states are observable