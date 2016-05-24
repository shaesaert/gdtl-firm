#!/usr/bin/env python
license_text='''
    Module implements the Kalman filters.
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

import numpy as np
# from numpy.linalg import inv, solve
from scipy.linalg import solve_discrete_are as dare

from linearsystem import LinearSystem


class KalmanFilter(object):
    '''The base class for Kalman filter implementations.'''
    
    def __init__(self, observationModel, motionModel):
        self.observationModel = observationModel # the observation model
        self.motionModel = motionModel # the motion model
    
    def predict(self, belief, control, ls):
        '''Gets as input belief and control, returns predicted belief if control
        were to be applied to the robot. Also called the Prior.
        (Eq. 75-76, 98-99)
        x^-_k+1 = A * x^+_k + B * u
        P^-_k+1 = A * P^+_k *A^T + G Q G^T
        '''
        A, bCov, G, Q = map(np.mat, [ls.getA(), belief.cov, ls.getG(), ls.getQ()])
        predictedState = self.motionModel.evolve(belief, control) # no noise
        predictedState.cov = np.asarray(A * bCov * A.T + G * Q * G.T)
        return predictedState
    
    def update(self, belief, obs, ls):
        '''Gets as input belief and observation, returns the updated state of
        the robot. Also called the Posterior.
        (Eq. 77-79, 100-102)
        K_k = P^-_k * H^T (H * P^-_k * H^T + M * R * M^T)^-1
        x^+_k+1 = x^-_k+1 + K_k+1 ( \delta z_k+1 - H * e^-_k+1)
        P^+_k+1 = (I - K_k+1 * H) P^-_k+1
        '''
        innov = np.mat(self.computeInnovation(belief, obs, ls)).T
        covPred, H, R, M = map(np.mat, [belief.cov, ls.getH(), ls.getR(), ls.getM()])
        KalmanGain = covPred * H.T * (H * covPred * H.T + M * R * M.T).I
        
#         print 'Cov Pred:\n', covPred
#         print 'R:\n', R
#         print 'M:\n', M
#         print 'H:\n', H
#         print 'KalmanGain:\n', KalmanGain
        
        xPredVec = np.mat(belief.conf).T
        xEstVec = xPredVec + KalmanGain * innov
        updatedState = belief.copy()
        updatedState.conf[:] = np.asarray(xEstVec).flatten()
        updatedState.cov[:, :] = covPred - KalmanGain * H * covPred
        return updatedState
    
    def evolve(self, belief, control, obs, lsPred, lsUpdate):
        '''Evolves the robot's belief on the input control, previous state and
        new observation. It first calls predict and then update.'''
        raise NotImplementedError
    
    def computeInnovation(self, state, obs, ls):
        '''Computes the innovation between observations that are predicted for
        the given state and the true observation.
        '''
        raise NotImplementedError
    
    def computeStationaryCovariance(self, ls):
        '''Computes the asymptotic covariance for a given linear system.'''
        raise NotImplementedError


class LinearizedKF(KalmanFilter):
    '''Class for Linearized Kalman Filter.'''
    
    def __init__(self, observationModel, motionModel):
        super(LinearizedKF, self).__init__(observationModel, motionModel)
    
    def evolve(self, belief, control, obs, lsPred, lsUpdate):
        '''Evolves the robot's belief on the input control, previous state and
        new observation. It first calls predict and then update.
        '''
        bPred = self.predict(belief, control, lsPred)
        if obs is None:
            return bPred.copy()
        return self.update(bPred, obs, lsUpdate)
    
    def computeInnovation(self, state, obs, ls):
        return obs - ls.getH().dot(state.conf)
    
    def computeStationaryCovariance(self, ls):
        '''Compute the asymptotic covariance for a given linear system.
        (Eq. 104-106)
        '''
        A, H, G, Q, M, R = map(np.mat, [ls.getA(), ls.getH(), ls.getG(),
                                        ls.getQ(), ls.getM(), ls.getR()])
        try:
            Pprd = np.mat(dare(A.T, H.T, G * Q * G.T, M * R * M.T))
        except:
            raise AssertionError("Dare Unsolvable: "
                          + "The given system state is not stable/observable.")
        assert np.all(np.isreal(Pprd)) and np.all(Pprd == Pprd.T)
        Pest = Pprd - (Pprd * H.T) * (H * Pprd * H.T + M * R * M.T).I * (Pprd * H.T).T
        assert np.all(np.isreal(Pest)) and np.all(Pest == Pest.T)
        return Pest


class ExtendedKF(KalmanFilter):
    '''Class for Extended Kalman Filter.'''
    
    def __init__(self, observationModel, motionModel):
        super(ExtendedKF, self).__init__(observationModel, motionModel)
    
    def evolve(self, belief, control, obs, lsPred, lsUpdate):
        '''Evolves the robot's belief on the input control, previous state and
        new observation. It first calls predict and then update.
        '''
        # the EKF does not use the linear systems passed to the filter, instead
        # it generates the linear systems on the fly
        lsPredicted = LinearSystem(belief, control, self.motionModel,
                                   self.observationModel)
        bPred = self.predict(belief, control, lsPredicted)
        if obs is None:
            return bPred.copy()
        lsUpdated = LinearSystem(bPred, control, self.motionModel,
                                 self.observationModel)
        return self.update(bPred, obs, lsUpdated)
    
    def computeInnovation(self, state, obs, ls):
        return obs - self.observationModel.getObservationPrediction(state)
