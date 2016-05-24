license_text='''
    Module defines the linear systems base class.
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

class LinearSystem(object):
    '''The linear system class.
    A Linear System is a construct which is used to store information about the
    system at a given state in the open loop trajectory. It is used to compute
    and retrieve the Jacobians at the particular state.
    '''
    
    __slots__ = ('motionModel', 'observationModel', 'x', 'u')
    
    def __init__(self, state=None, control=None,
                 motionModel=None, observationModel=None):
        '''Constructor.'''
        self.motionModel = motionModel # the motion model that models the state transition
        self.observationModel = observationModel # the observation model that models the sensor measurement
        
        self.x = state.copy() # the state at which the linear system is constructed
        self.u = control # the control applied at the internal state
    
    def getA(self):
        '''Get the state transition Jacobian.'''
        return self.motionModel.getStateJacobian(self.x, self.u)
    
    def getB(self):
        '''Get the control Jacobian for the state transition.'''
        return self.motionModel.getControlJacobian(self.x, self.u)
    
    def getG(self):
        '''Get the noise Jacobian in the state transition.'''
        return self.motionModel.getNoiseJacobian(self.x, self.u)
    
    def getQ(self):
        '''Get the process noise covariance for the state transition.'''
        return self.motionModel.processNoiseCovariance(self.x, self.u)
    
    def getH(self):
        '''Get the Jacobian for the observation.'''
        return self.observationModel.getObservationJacobian(self.x)
    
    def getM(self):
        '''Get the observation noise Jacobian.'''
        return self.observationModel.getNoiseJacobian(self.x)
    
    def getR(self):
        '''Get the observation noise covariance.'''
        return self.observationModel.getObservationNoiseCovariance(self.x)
