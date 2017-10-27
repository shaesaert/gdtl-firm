license_text='''
    Module deals with SE2 belief state spaces and sampling.
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
from numpy import pi
from numpy.linalg import norm


class E2StateSampler(object):
    '''Class is used to generate random samples in E(2).'''

    def __init__(self, bounds):
        self.low, high = np.array(bounds).reshape((2, 2))
        self.extend = high - self.low

    def sample(self, freeze=True):
        x, y = self.low + np.random.rand(2) * self.extend
        return E2BeliefState(x, y, freeze=freeze)


class SE2StateSampler(object):
    '''Class is used to generate random samples in SE(2).'''

    def __init__(self, bounds):
        self.low, high = np.array(bounds).reshape((2, 2))
        self.extend = high - self.low

    def sample(self, freeze=True):
#         assert False #TODO: ????
        x, y = self.low + np.random.rand(2) * self.extend
        theta = -pi + np.random.rand(1) * 2 *pi
        return SE2BeliefState(x, y, theta, freeze=freeze)


class E2BeliefState(object):
    '''A belief in E(2): (x, y, covariance)'''
    __slots__ = ('conf', 'cov', 'frozen', '_hash')

    reachDist = None
    normWeights = None
    meanNormWeight = None
    covNormWeight = None

    def __init__(self, x=0, y=0, cov=None, freeze=False):
        self.conf = np.array([x, y], dtype=np.float)
        if cov is None:
            self.cov = np.zeros((2, 2))
        else:
            self.cov = np.array(cov)

        if freeze:
            self._hash = hash((tuple(self.conf), tuple(self.cov.flatten())))
        self.frozen = freeze

    def getConf(self):
#         return np.array(self.conf)
        return self.conf

    def setConf(self, conf):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[:] = conf

    def freeze(self):
        if not self.frozen:
            self.frozen = True
            self._hash = hash((tuple(self.conf), tuple(self.cov.flatten())))

    def thaw(self):
        self.frozen = False
        del self._hash

    @property
    def x(self):
        return self.conf[0]
    @x.setter
    def x(self, value):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[0] = value
    @property
    def y(self):
        return self.conf[1]
    @y.setter
    def y(self, value):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[1] = value

    def isReached(self, state):
        '''Checks if the input state has stabilized to this state (node
        reachability check).
        '''
        # subtract the two beliefs and get the norm
        stateDiff = self.conf - state.conf

        covDiff = state.cov - self.cov
        covDiffDiag = np.sqrt(np.maximum(np.diag(covDiff), 0))

        # Need weighted supNorm of difference in means
        meanNorm = norm(stateDiff * E2BeliefState.normWeights, ord=np.inf)
        covDiagNorm = norm(covDiffDiag * E2BeliefState.normWeights, ord=np.inf)
        norm2 = max(meanNorm * E2BeliefState.meanNormWeight,
                    covDiagNorm * E2BeliefState.covNormWeight)
        return norm2 <= E2BeliefState.reachDist

    def distanceTo(self, state):
        return norm(self.conf[:2] - state.conf[:2], ord=2)

    def copy(self, freeze=False):
        return E2BeliefState(self.x, self.y, self.cov, freeze=freeze)

    def setState(self, state):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[:] = state.x, state.y
        self.cov[:, :] = state.cov

    def __eq__(self ,other):
        return np.all(self.conf == other.conf) and np.all(self.cov == other.cov)

    def __neq__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        assert self.frozen
        return self._hash

    def __str__(self):
        return 'State [X, Y]: [{0}, {1}]\nCovariance:\n{2}'.format(
                                            self.x, self.y, self.cov)

    __repr__ = __str__


class SE2BeliefState(object):
    '''A belief in SE(2): (x, y, yaw, covariance)'''
    __slots__ = ('conf', 'cov', 'frozen', '_hash')

    reachDist = None
    normWeights = None
    meanNormWeight = None
    covNormWeight = None

    def __init__(self, x=0, y=0, yaw=0, cov=None, freeze=False):
        self.conf = np.array([x, y, self.normalizeAngle(yaw)], dtype=np.float)
        if cov is None:
            self.cov = np.zeros((3,3))
        else:
            self.cov = np.array(cov)

        if freeze:
            self._hash = hash((tuple(self.conf), tuple(self.cov.flatten())))
        self.frozen = freeze

    @classmethod
    def normalizeAngle(cls, theta):
        '''Normalizes angle to between -pi and pi'''
        return theta - (np.ceil((theta + pi)/(2*pi)) - 1) * 2 * pi

    def getConf(self):
#         return np.array(self.conf)
        return self.conf

    def setConf(self, conf):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[:] = conf
        self.conf[2] = self.normalizeAngle(self.conf[2])

    def freeze(self):
        if not self.frozen:
            self.frozen = True
            self._hash = hash((tuple(self.conf), tuple(self.cov.flatten())))

    def thaw(self):
        self.frozen = False
        del self._hash

    @property
    def x(self):
        return self.conf[0]
    @x.setter
    def x(self, value):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[0] = value
    @property
    def y(self):
        return self.conf[1]
    @y.setter
    def y(self, value):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[1] = value
    @property
    def yaw(self):
        return self.conf[2]
    @yaw.setter
    def yaw(self, value):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[2] = self.normalizeAngle(value)

    def isReached(self, state):
        '''Checks if the input state has stabilized to this state (node
        reachability check).
        '''
        # subtract the two beliefs and get the norm
        stateDiff = self.conf - state.conf
        stateDiff[2] = self.normalizeAngle(stateDiff[2])
        assert -pi <= stateDiff[2] <= pi
        
        covDiff = state.cov - self.cov
        covDiffDiag = np.sqrt(np.maximum(np.diag(covDiff), 0))
        
        # Need weighted supNorm of difference in means #FIXME: which one to use?
#         meanNorm = norm(stateDiff * SE2BeliefState.normWeights, ord=np.inf)
        meanNorm = norm(stateDiff * SE2BeliefState.normWeights, ord=2)
        covDiagNorm = norm(covDiffDiag * SE2BeliefState.normWeights, ord=np.inf)
        norm2 = max(meanNorm * SE2BeliefState.meanNormWeight,
                    covDiagNorm * SE2BeliefState.covNormWeight)
        
#         print '[isReached]', norm2, SE2BeliefState.reachDist, norm2 <= SE2BeliefState.reachDist
        
        return norm2 <= SE2BeliefState.reachDist

    def distanceTo(self, state):
        return norm(self.conf[:2] - state.conf[:2], ord=2)

    def copy(self, freeze=False):
        return SE2BeliefState(self.x, self.y, self.yaw, self.cov, freeze=freeze)

    def setState(self, state):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[:] = state.x, state.y, state.yaw
        self.cov[:, :] = state.cov

    def __eq__(self ,other):
        return np.all(self.conf == other.conf) and np.all(self.cov == other.cov)
    
    def __neq__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        assert self.frozen
        return self._hash

    def __str__(self):
        return 'State [X, Y, Yaw]: [{0}, {1}, {2}]\nCovariance:\n{3}'.format(
                                            self.x, self.y, self.yaw, self.cov)

    __repr__ = __str__
