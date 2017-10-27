license_text='''
    Module deals with the rover belief state space (SE2 + layered map) and
    sampling.
    Copyright (C) 2017  Cristian Ioan Vasile <cvasile@bu.edu>
    CSAIL, LIDS, Massachusetts Institute of Technology

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

from se2beliefspace import SE2BeliefState


class SE2ShadowStateSampler(object):
    '''Class is used to generate random samples in SE(2).'''

    def __init__(self, bounds, maps):
        self.low, high = np.array(bounds).reshape((2, 2))
        self.extend = high - self.low
        self.map = maps

    def sample(self, freeze=True):
#         assert False #TODO: ????
        while True:
            x, y = self.low + np.random.rand(2) * self.extend
            theta = -pi + np.random.rand(1) * 2 *pi
            if not self.map.value((x, y, theta), 'shadow'):
                return SE2BeliefState(x, y, theta, freeze=freeze)

class Map(object):
    '''Class represents a layered map of the environment. Each layer corresponds
    to a type of feature that is tracked in the environment, e.g., rocks, sand,
    shadow (explored area), samples of different types.
    '''
    
    def __init__(self, bounds, resolution, layer_priors, collectables,
                 dtype=np.bool):
        '''Constructor

        Arguments
        ---------

        bounds: [[x_low, y_low], [x_high, y_high]]
            The bounds of the environment.

        resolution: float
            The step size used to produce the grid, i.e., the resolution of the
            layers.

        layer_priors: dictionary
            The map containing the priors used to define and initialize the
            map's layers.

        collectables: iterable
            Iterable data structure, containing the labels of samples that can
            be collected.
        
        dtype: numpy data type or dictionary (default: numpy.bool)
            The numpy data type used for the map's layers.
        
        FIXME: The implementation assumes that dtype is np.bool.
        '''
#         # the set of collectable items must be a subset of the layers types
#         assert set(collectables) <= set(layer_priors)
        
        self.step = float(resolution)

        if not isinstance(dtype, dict):
            dtype = {label:dtype for label in layer_priors}

        (x_low, y_low), (x_high, y_high) = bounds

        self.layers = dict()
        for label, prior in layer_priors.iteritems():
            ncols = int(np.ceil((x_high - x_low)/self.step))
            nrows = int(np.ceil((y_high - y_low)/self.step))
            
            # use center of each cell in the grid to set their values
            self.layers[label] = np.array([[prior((x, y, 0))
                for x in np.arange(x_low + self.step/2, x_high, self.step)]
                    for y in np.arange(y_low + self.step/2, y_high, self.step)],
                                          dtype=dtype[label])
            assert self.layers[label].shape == (nrows, ncols)
        self.bounds = bounds
        self.collectables = collectables

    def check_bounds(self, x, y):
        '''Checks if the 2d point (x, y) is within the map's boundary.'''
        return (self.bounds[0][0] < x < self.bounds[1][0]
                and self.bounds[0][1] < y < self.bounds[1][1])

    def value(self, state, label):
        '''TODO: add description
        '''
        x, y, _ = state
        assert self.check_bounds(x, y)
        px = (x - self.bounds[0][0])/self.step
        py = (y - self.bounds[0][1])/self.step
        return self.layers[label][py][px]

    def copy(self):
        '''Returns a (deep) copy of the layered map.
        Note: The collectables attribute is not copied, instead a reference is
        passed to the new object. 
        '''
        map_ = Map(self.bounds, self.step, dict(), ())
        for label, layer in self.layers.iteritems():
            map_.layers[label] = layer.copy()
        map_.collectables = self.collectables
        return map_


class RoverBeliefState(object):
    '''A belief in SE(2) x Map: (x, y, yaw, covariance, map)'''
    __slots__ = ('conf', 'cov', 'map', 'frozen', '_hash')

    reachDist = None
    normWeights = None
    meanNormWeight = None
    covNormWeight = None

    def __init__(self, x=0, y=0, yaw=0, cov=None, map_=None, freeze=False):
        self.conf = np.array([x, y, self.normalizeAngle(yaw)], dtype=np.float)
        if cov is None:
            self.cov = np.zeros((3,3))
        else:
            self.cov = np.array(cov)
        
        if map_ is None:
            pass
        else:
            self.map = map_.copy()

        if freeze:
            self._hash = hash((tuple(self.conf), tuple(self.cov.flatten()),
                tuple((label, tuple(layer.flatten))
                    for label, layer in self.map.layers.iter_iterms)))
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
            self._hash = hash((tuple(self.conf), tuple(self.cov.flatten()),
                          tuple((label, tuple(layer.flatten))
                              for label, layer in self.map.layers.iter_iterms)))

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
        meanNorm = norm(stateDiff * RoverBeliefState.normWeights, ord=2)
        covDiagNorm = norm(covDiffDiag * RoverBeliefState.normWeights, ord=np.inf)
        norm2 = max(meanNorm * RoverBeliefState.meanNormWeight,
                    covDiagNorm * RoverBeliefState.covNormWeight)
        
#         print '[isReached]', norm2, SE2BeliefState.reachDist, norm2 <= SE2BeliefState.reachDist
        
        return norm2 <= RoverBeliefState.reachDist

    def distanceTo(self, state):
        return norm(self.conf[:2] - state.conf[:2], ord=2)

    def copy(self, freeze=False):
        return RoverBeliefState(self.x, self.y, self.yaw, self.cov, self.map,
                                freeze=freeze)

    def setState(self, state):
        if self.frozen:
            raise TypeError('Can not modify frozen object!')
        self.conf[:] = state.x, state.y, state.yaw
        self.cov[:, :] = state.cov
        self.map = state.map.copy()

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
