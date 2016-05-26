#!/usr/bin/env python
license_text='''
    Module implements GDTL-FIRM algorithm.
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
import time
import itertools as it
from collections import namedtuple, deque, defaultdict, Counter

import numpy as np
from numpy.random import multivariate_normal as mv_gauss
from scipy.spatial import cKDTree as NearestNeightbor
import networkx as nx
import yaml

from lomap import Ts
from lomap import Markov as MDP
from lomap import Rabin
from lomap import Timer
from gdtl import gdtl2ltl, PredicateContext

from linearsystem import LinearSystem
from controller import SwitchingController
from controller import NullController
# from controller import SLQRController, TrackingLQRController
from controller import Controller
from filter import LinearizedKF, ExtendedKF


Solution = namedtuple('Solution', ['path', 'prob'])


class FStrategy(object):
    '''Class implements a connection strategy based on nearest neighbors.
    The class keeps track of states using a k-d tree which is polled for nearest
    states and is needs to be updated whenever new states are added to the
    transition system.
    '''
    def __init__(self, no_neighbors, radius):
        # nearest neighbors data structure
        self.nn = None
        self.max_no_neighbors = no_neighbors
        self.max_dist = radius
        self.states = [] # TODO: can I get rid of this? maybe take it from ts
    
    def add(self, state):
        self.states.append(state)
        self.nn = NearestNeightbor([s.conf[:2] for s in self.states])
    
    def nearest(self, state, ret_dist=False):
        s, d = None, -1
        if self.nn:
            idx = self.nn.query(state.conf[:2])
            d, idx = idx
            assert idx == int(idx)
            p = self.nn.data[idx]
            s = self.states[idx]
            assert np.all(s.conf[:2] == p)
        if ret_dist:
            return s, d
        return s
    
    def near(self, state):
        if self.nn:
            _, idxs = self.nn.query(state.conf[:2], k=self.max_no_neighbors,
                                    distance_upper_bound=self.max_dist)
            return [self.states[idx] for idx in idxs if idx < len(self.states)]
        return


class FIRM(object):
    '''Feedback Information RoadMap planner'''
    
    def __init__(self, env, motion_model, observation_model, parameters):
        '''Constructor'''
        self.name = "LTL FIRM Planner"
        
        self.bounds = np.array(env['boundary']['limits']).reshape(2, 2)
        self.motion_model = motion_model
        self.observation_model = observation_model
        
        # sampler used for generating valid samples in the state space
        self.sampler = None
        
        nn = parameters['nearest_neighbors']
        self.connection_strategy = FStrategy(nn['max_number_of_neighbors'],
                                             nn['max_connection_radius'])
        self.min_steering_dist, self.steering_radius = parameters['steering_dist_bounds']
        
        # TS and Product MDP
        self.ts = Ts(multi=False)
        self.pa = MDP(multi=False)
        self.pa.rabin_states = defaultdict(set)
        # set specification
        self.specification = None
        # number of Monte Carlo simulations
        self.numParticles = parameters['number_monte_carlo_trials']
        
        # state and control weights for the LQR controllers
        cc = parameters['controller'] 
        self.state_weight = np.array(cc['state_weight'])
        self.control_weight = np.array(cc['control_weight'])
        assert np.all(np.linalg.eigvals(self.state_weight) > 0) # check pdef
        assert np.all(np.linalg.eigvals(self.control_weight) > 0) # check pdef
        
        self.cnt = it.count()
    
    def setSpecification(self, specification, state_label='x', cov_label='P'):
        '''Performs several operations to setup the planner's specification. The
        specification given as a GDTL formula is augmented with the boundary
        condition. Afterwards, its is translates to LTL, and the bijection
        between atomic propositions and predicates is computed. The obtained LTL
        specification is converted to a Deterministic Rabin Automaton. Lastly,
        the predicate evaluation context is created and saved.
        '''
        if specification is None:
            return
        
        self.specification = specification
        # add boundary constraint to specification
        self.specification += ' && G (norm({}) <= 1)'.format(state_label)
        # convert GDTL specification to LTL
        self.tl, self.ap = gdtl2ltl(formula=self.specification)
        # compute Rabin automaton from LTL formula 
        self.rabin = Rabin(multi=False)
        ltl_formula = self.tl.formulaString(ltl=True)
        self.rabin.rabin_from_formula(formula=ltl_formula)
        logging.info('LTL formula from GDTL: %s', ltl_formula)
        logging.info('Rabin size: (%d, %d)', self.rabin.g.number_of_nodes(),
                     self.rabin.g.number_of_edges())
        logging.info('Predicate to AP map: %s', self.ap)
        # set predicate evaluation context
        low, high = self.bounds
        extend = high - low
        xc = (low + high)/2.0
        self.context = PredicateContext(
                {'norm': lambda x: np.linalg.norm(np.array(x[:2]-xc)/extend,
                                                  ord=np.infty)})
        # save state and covariance labels
        self.state_label = state_label
        self.covariance_label = cov_label
    
    def getAPs(self, belief):
        '''Computes the set of atomic propositions that are true at the given
        belief node.
        '''
        # update predicate evaluation context with custom symbols
        return set([ap for ap, pred in self.ap.iteritems()
                           if self.context.evalPred(pred,
                                       belief.conf[:2], belief.cov[:2, :2], #TODO: make this general
                                       state_label=self.state_label,
                                       cov_label=self.covariance_label)])
    
    def addState(self, state, initial=False, copy=True):
        '''Adds a state to the transition system.
        Note: By default, a frozen copy of the state is added to the transition
        system. 
        '''
        # construct node controller and update state
        controller = self.generateNodeController(state)
        logging.debug('New belief node: %s, diag(cov): %s', state.conf,
                                                            np.diag(state.cov))
        if copy:
            s = state.copy(freeze=True)
        else:
            s = state
            s.freeze()
        # compute set of atomic propositions
        props = self.getAPs(state)
#         print props, self.ap
        self.ts.g.add_node(s, attr_dict = {'prop': props, 
                              'controller': controller, 'id': self.cnt.next()})
        if initial:
            self.ts.init[s] = 1
        # update the connection strategy
        self.connection_strategy.add(s)
    
    def processTrajectory(self):
        traj = deque()
        while True:
            state = yield traj
            props = self.getAPs(state)
            traj.append((state, props))
    
    def addTransition(self, a, b):
        '''Add a transition from state a to b in the transition system.'''
        controller = self.generateEdgeController(a, b)
        nc = self.ts.g.node[b]['controller']
        with Timer('compute trajectories for edge'):
            randomStartStates = [a.copy() for _ in xrange(self.numParticles)]
            [r.setConf(mv_gauss(mean=a.conf, cov=a.cov)) for r in randomStartStates]
            
            processTrajectories = [self.processTrajectory() for _ in xrange(self.numParticles)]
            map(next, processTrajectories) # initialize trajectory processing
            #TODO: add edge controller
            trajectories = [nc.execute(r, pt)[1]
                   for r, pt in it.izip(randomStartStates, processTrajectories)]
        self.ts.g.add_edge(a, b, attr_dict={'controller': controller,
                                            'trajectories': trajectories})
    
    def generateNodeController(self, state): #, nodeController):
        '''Generates the node controller that stabilizes the robot at the node'''
        # construct a linear Kalman filter
        lkf = LinearizedKF(self.observation_model, self.motion_model)
        if self.observation_model.isStateObservable(state):
            # construct a linear system
            ls = LinearSystem(state, self.motion_model.zeroControl,
                              self.motion_model, self.observation_model)
            # compute the stationary cov at node state using LKF
            stationaryCovariance = lkf.computeStationaryCovariance(ls)
        else:
            raise AssertionError('Assumes that all states are observable!')
        # set the covariance
        state.cov = stationaryCovariance
        # create the node controller: SLQR + LKF
#         lqr = OffAxisSLQRController(self.motion_model, self.observation_model,
#                              state, self.state_weight, self.control_weight) #TODO: test this
#         lqr = SLQRController(self.motion_model, self.observation_model,
#                              state, self.state_weight, self.control_weight)
        lqr = SwitchingController(self.motion_model, self.observation_model,
                                  state, self.state_weight, self.control_weight)
        return Controller(self.motion_model, self.observation_model, lqr, lkf)
    
    def generateEdgeController(self, start, target):
        '''Generates the edge controller that drives the robot from the start
        node to the target node of edge.
        '''
        assert (start is not None) and (target is not None)
        ekf = ExtendedKF(self.observation_model, self.motion_model)
#         lqr = TrackingLQRController(self.motion_model, self.observation_model,
        lqr = NullController(self.motion_model, self.observation_model, #TODO: remove this if it does nothing
                                    transition=(start, target),
                                    stateWeight=self.state_weight,
                                    controlWeight=self.control_weight)
        # create the edge controller: Tracking LQR + EKF
        return Controller(self.motion_model, self.observation_model, lqr, ekf)
    
    def canSteer(self, src, dest):
        '''Checks if the system can steer from the source state to the
        destination state.
        '''
        # assumes that steering is always possible
        return True
    
    def steer(self, src, dest):
        '''Steers the system from the source state to the destination state.'''
        dist = src.distanceTo(dest)
        if dist > self.steering_radius:
            s, d = src.conf, dest.conf
            r = s + (self.steering_radius/dist) * (d - s)
            new = dest.copy()
            new.conf[:] = r
            new.freeze()
            return new
        return dest
    
    def extend(self):
        '''Generate a state near one in the transition system.'''
        while True: # check if it is already in ts
            random_state = self.sampler.sample() # generate new sample
            # find nearest state in transition system
            state, d = self.connection_strategy.nearest(random_state, True)
#             print 'extend: nearest:', state, d
            if d > self.min_steering_dist:
                break
        # steer towards the random state
        return self.steer(src=state, dest=random_state)
    
    def initPA(self):
        '''Initialized the Product MDP.'''
        assert len(self.ts.init) == 1
        assert self.rabin is not None
        
        if not self.pa.init:
            ts_init = next(self.ts.init.iterkeys())
            rabin_init = next(self.rabin.init.iterkeys())
            self.pa.init[(ts_init, rabin_init)] = 1
        else:
            ts_init, rabin_init = next(self.pa.init.iterkeys())
        if not self.pa.rabin_states.has_key(ts_init):
            self.pa.rabin_states[ts_init] = set([rabin_init])
        # initialize SCCs
        self.initSCCs()
    
    def trajectoryAnnotation(self, s_u, trajectory):
        edge = np.zeros((len(self.rabin.final), 2), np.bool)
        s_v = s_u
        for _, props in trajectory:
            next_rabin_states = self.rabin.next_states_of_rabin(s_v, props)
            assert len(next_rabin_states) == 1
            s_v = next_rabin_states[0]
            edge |= [(s_v in F, s_v in B) for F, B in self.rabin.final]
        return edge, s_v
    
    def computeProbabilities(self, start, s_u, target):
        '''Computes reachability and satisfaction probabilities of edges in the
        product MDP.
        '''
        tps = Counter() # transition probabilities for edges in PA
        sps = dict() # satisfaction probabilities for edges in PA
        
        for traj in self.ts.g[start][target]['trajectories']:
            edge, s_v = self.trajectoryAnnotation(s_u, traj)
            tps[s_v] += 1
            if sps.has_key(s_v):
                sps[s_v] += edge
            else:
                sps[s_v] = edge.astype(np.float)
        npart = self.numParticles
        pa_start = (start, s_u)
        # returns Rabin state and PA transition sets
        return (tps.keys(),
                [(pa_start, (target, s_v),
                  {'prob': tp/float(npart), 'sat': sps[s_v]/npart})
                                                for s_v, tp in tps.iteritems()])
    
    def updatePA(self, u, v):
        '''Updates the product automaton using the TS transition (u, v).'''
        assert u in self.pa.rabin_states
        
#         print 'pre update:', u.conf, v.conf
#         for ts_state, r_state in self.pa.g.nodes_iter():
#             print 'node:', (ts_state.conf, r_state)
#         print
#         for (qu, s_qu), (qv, s_qv) in self.pa.g.edges_iter():
#             print 'edge:', (qu.conf, s_qu), (qv.conf, s_qv)
#         print
        
        stack = deque([(u, s_u, v) for s_u in self.pa.rabin_states[u]])
#         delta = []
        while stack:
#             print stack
            start, s_u, target = stack.popleft()
#             print 'current stack pop:', start.conf, s_u, target.conf
#             print
            with Timer('compute probabilities'):
                r_states, transitions = self.computeProbabilities(start, s_u, target)
            assert r_states
            assert transitions
#             print r_states
#             for (qu, s_qu), (qv, s_qv), d in transitions:
#                 print 'transition:', (qu.conf, s_qu), (qv.conf, s_qv), d
#             print
            # update PA map from ts states to rabin states
            self.pa.rabin_states[target].update(r_states)
#             print r_states, self.pa.rabin_states[target]
#             for (x1, s1), (x2, s2), d in transitions:
#                 print (x1.conf, s1), (x2.conf, s2), d
#             print
#             print
            # add new states to stack for processing
            for r_state in r_states:
                if (target, r_state) not in self.pa.g:
                    stack.extend([(target, r_state, nb)
                                    for nb in self.ts.g.neighbors_iter(target)])
            # add transitions to the PA
            self.pa.g.add_edges_from(transitions)
            with Timer('update SCCs'):
                self.updateSCCs(transitions)
#             delta.extend(transitions)
#         print 'post update:'
#         for ts_state, r_state in self.pa.g.nodes_iter():
#             print 'node:', (ts_state.conf, r_state)
#         print
#         for (qu, s_qu), (qv, s_qv) in self.pa.g.edges_iter():
#             print 'edge:', (qu.conf, s_qu), (qv.conf, s_qv)
#         print
#         print
#         print
#         print
#         # update PA SCCs
#         self.updateSCCs(delta)
    
    def initSCCs(self):
        '''Initializes the SCCs for each F,B pair in the acceptance set.'''
        self.pa.subgraphs = [nx.DiGraph() for _ in self.rabin.final]
        self.pa.sccs = [[] for _ in self.rabin.final]
        self.pa.good_transitions  = [[] for _ in self.rabin.final]
        self.pa.good_sccs = [[] for _ in self.rabin.final]
    
    def updateSCCs(self, transitions):
        '''Updates the subgraphs and SCCs associated with each pair in the
        acceptance set.
        
        Note: Assumes that all transitions correspond to a single TS transition.
        '''
        # process transitions and update SCCs based on intersection probabilities
        for k, subg in enumerate(self.pa.subgraphs):
            # check if any of the transitions intersect the k-th bad set
            if all([(d['sat'][k][1] == 0) for _, _, d in transitions]):
                sub_trans = [(u, v, {'prob': d['prob']})
                                                    for u, v, d in transitions]
                subg.add_edges_from(sub_trans) # add all transitions to subgraph
                good_trans = [(u, v) for u, v, d in transitions
                                         if d['sat'][k][0] > 0]
                self.pa.good_transitions[k].extend(good_trans)
#         for u, v, d in transitions: #TODO: check this
#             for k, pathProb in enumerate(d['sat']):
#                 if not pathProb[1]: # does not intersect the bad set
#                     self.pa.subgraphs[k].add_edge(u, v, prob=d['prob'])
#                 if pathProb[0]: # intersects the good set, save it
#                     self.pa.good_transitions[k].append((u, v))
        # update SCCs
        for k, subg in enumerate(self.pa.subgraphs): #TODO: check this
            self.pa.sccs[k] = map(tuple, nx.strongly_connected_components(subg))
        # compute good SCCs
        self.pa.good_sccs = [set() for _ in self.rabin.final]
        for trs, sccs, gsccs in it.izip(self.pa.good_transitions, self.pa.sccs, self.pa.good_sccs):
            for u, v in trs:
                gsccs.update(tuple([scc for scc in sccs if (u in scc) and (v in scc)]))
    
    def existsSatisfyingRun(self):
        '''Checks if the constructed MDP contains a satisfying run of non-zero
        probability.  # TODO: check this
        '''
        if self.found:
            return True
        if any(self.pa.good_sccs):
            self.found = True
        return self.found
    
    def solveDP(self):
        '''Solve the DP end component reachability problem.'''
        def recursion(state, k, cost_to_go, optimal_actions):
            if not self.pa.good_sccs[k]:
                return 0 # there is no good end component
            if state in cost_to_go:
                assert state in optimal_actions
                return cost_to_go[state]
            good_sccs_state = [scc for scc in self.pa.good_sccs[k] if state in scc]
            assert len(good_sccs_state) <= 1, (len(good_sccs_state), good_sccs_state)
            if good_sccs_state:
                cost_to_go[state] = 1
                nbs = (nb for nb in self.pa.subgraphs[k].neighbors_iter(state)
                                                    if nb in good_sccs_state[0])
                print 'State in good SCCs:', state, nbs.next()[0]
                optimal_actions[state] = nbs.next()[0] #TODO: may need to create paths inside end component
            else:
                costs = dict()
                for _, nb, d in self.pa.g.edges_iter([state], data=True):
                    nb_cost = recursion(nb, k, cost_to_go, optimal_actions)
                    costs[nb[0]] = costs.get(nb[0], 0) + d['prob'] * nb_cost
                action = max(costs, key=costs.get)
                print 'State not in good SCCs:', state, action
                optimal_actions[state] = action
                cost_to_go[state] = costs[action]
            return cost_to_go[state]
        
        opt_cost_to_go = -1
        optimal_actions = None
        init_pa_state = iter(self.pa.init).next()
        for k in range(len(self.rabin.final)):
            print 'Pair', k
            costs_map = dict()
            actions_map = dict()
            # compute cost to go from initial state and populate costs and actions maps
            init_cost_to_go = recursion(init_pa_state, k, costs_map, actions_map)
            print 'solveDP', costs_map
            print 'solveDP', actions_map
            if init_cost_to_go > opt_cost_to_go:
                opt_cost_to_go = init_cost_to_go
                optimal_actions = actions_map # Note: reference copy only
        
        return Solution(optimal_actions, prob=opt_cost_to_go)
    
    def solve(self, steps=1, maxtime=300, epsilon=0.01): 
        '''Solves the motion planning problem.'''
        if not self.ts.init:
            return False
        self.initPA()
        
        self.solution = Solution(None, prob=-1)
        self.found = False
        
        logging.info("%s: Initial state:\n%s", self.name, next(self.ts.init.iterkeys()))
        
        nrStartStates = self.ts.g.number_of_nodes()
        logging.info("%s: Starting with %u states", self.name, nrStartStates)
        
        t0 = time.time()
        logging.info("%s: starting time %f ", self.name, t0)
        
        trtime = []
        
        for _ in xrange(steps):
            if time.time() - t0 > maxtime:
                logging.info("%s: reached maximum allowed time", self.name)
                return False
            
            xn = self.extend()
            near_states = self.connection_strategy.near(xn)
            assert len(near_states) > 0
            transitions = [(x, xn) for x in near_states if self.canSteer(x, xn)]
            transitions += [(xn, x) for x in near_states if self.canSteer(xn, x)]
            
            self.addState(xn, copy=False)
            
            
            for u, v in transitions:
                t1 = time.time()
                self.addTransition(u, v)
                self.updatePA(u, v)
                trtime.append(time.time()-t1)
            
            if self.existsSatisfyingRun():
                pass
#                 # solve dynamic program to get MP satisfying policy
# #                 return True
#                 solution = self.solveDP()
#                 if self.solution.prob < solution.prob:
#                     self.solution = solution
#                 if self.solution.prob > epsilon:
#                     logging.info("%s: found solution with probability %f",
#                                  self.name, solution.prob)
#                     logging.info('[solve] actions:')
#                     logging.info('[solve] feedback policy: %s', self.solution.path)
#                     for (u, s_u), action in self.solution.path.iteritems():
#                         logging.info('state: %s -> action: %s', (u.conf, s_u), action.conf)
#                     return True
            print 'Found:', self.found
        
        print '>>>>'
        print 'Found:', self.found
#         print 'Prob:', self.solution.prob
#         print 'OptimalActions:', self.solution.path
        print 'TS:', self.ts.g.number_of_nodes(), self.ts.g.number_of_edges()
        print 'PA:', self.pa.g.number_of_nodes(), self.pa.g.number_of_edges()
        print 'mean trtime:', np.mean(trtime)
        print 'overall:', time.time() - t0
#         print 'ptime:', ptime
        logging.info("%s: reached maximum allowed steps", self.name)
        print '<<<<<'
        return False
    
    #---------------------------------------------------------------------------
    ## STUBS ##
    
    def executePolicy(self):
        '''Executes the computed feedback policy on the robot.''' #TODO: test
        return
        assert self.solution.path
        return
        states_id_map = [None] * self.ts.g.number_of_nodes()
        for s, d in self.ts.g.nodes_iter(data=True):
            states_id_map[d['id']] = s
        
        self.solution = Solution([states_id_map[i] for i in [0, 1, 2, 16, 20, 23, 5, 7, 17, 11, 14, 22, 8, 6, 0, 1, 2, 16, 20, 23, 5, 7, 17, 11, 14, 22, 8, 6, 0]], 1)
        print [states_id_map[i].conf for i in [0, 1, 2, 16, 20, 23, 5, 7, 17, 11, 14, 22, 8, 6, 0, 1, 2, 16, 20, 23, 5, 7, 17, 11, 14, 22, 8, 6, 0]]
        
#         import matplotlib.pyplot as plt
#         plt.figure()
#         x, y = zip(*[s.conf for s in self.solution.path])
#         plt.plot(x, y)
#         plt.figure()
# #         plt.show()
#         return
        
        self.motion_model.executionMode(isSimulation=False)
        self.observation_model.executionMode(isSimulation=False)
        
        # set initial state
        updatedState, rabin_init = next(self.pa.init.iterkeys())
        process = self.processTrajectory(rabin_init)
        process.next()
        for s, t in it.izip(self.solution.path[:-1], self.solution.path[1:]):
            ec = self.ts.g[s][t]['controller'] # edge controller
            nc = self.ts.g.node[t]['controller'] # node controller
            ec.withNoise = False
            nc.withNoise = False
            
            updatedState, _ = ec.execute(updatedState, process)
#             print 'UPD:', updatedState
#             assert nc.feedbackController.isTerminated(updatedState, 0)
            updatedState, data = nc.execute(updatedState, process)
        
        return updatedState, data
    
    def savePlannerData(self, filename):
        '''Save the models to a file'''
        ts_edges = [(tuple(map(float, u.conf)), tuple(map(float, v.conf)))
                                            for u, v in self.ts.g.edges_iter()]
        ts_init = tuple(map(float, next(self.ts.init.iterkeys()).conf))
#         pa_edges = [((tuple(map(float, u.conf)), s_u),
#                      (tuple(map(float, v.conf)), s_v),
#                      {'prob': float(d['prob']), 'sat': tuple(map(float, d['sat'].flatten()))})
#                     for (u, s_u), (v, s_v), d in self.pa.g.edges(data=True)]
#         pa_init = iter(self.pa.init).next()
#         pa_init = (tuple(map(float, pa_init[0].conf)), pa_init[1])
         
#         with open(filename, 'w') as fout:
#             yaml.dump({'ts': ts_edges, 'ts_init': ts_init,},
# #                        'pa': pa_edges, 'pa_init': pa_init,},
#                        fout)
         
#         for s in self.pa.g:
#             print 'Node:', s[0].conf, s[1], 'cov:', np.diag(s[0].cov)
#             print 'Neighbors:'
#             
#             for _, nb, d in self.pa.g.edges_iter([s], data=True):
#                 print '\t', nb[0].conf, nb[1], d['prob'], d['sat'].flatten()
#             
#             print
         
        print self.ts.g.number_of_nodes(), self.ts.g.number_of_edges()
        print self.pa.g.number_of_nodes(), self.pa.g.number_of_edges()
#         
#     
#     def loadRoadMapFromFile(self, pathToFile):
#         '''Load the models info from a file''' #TODO: maybe from YAML
#         with open(pathToFile, 'r') as fin:
#             data = yaml.load(fin)
#         
#         nodes = set([SE2BeliefState(u[0], u[1], freeze=True) for u, _ in data['ts']]
#                     + [SE2BeliefState(v[0], v[1], freeze=True) for _, v in data['ts']])
#         init_conf = iter(self.ts.init).next().conf
#         print 'nr nodes:', len(nodes)
#         for u in nodes:
#             if np.all(u.conf != init_conf):
#                 self.addState(u)
#         
#         edges = [(SE2BeliefState(u[0], u[1], freeze=True), SE2BeliefState(v[0], v[1], freeze=True))
#                     for u, v in data['ts']]
#         print self.ts.g.number_of_nodes()
#         for u, v in edges:
#             u_state = [uu for uu in self.ts.g if np.all(u.conf == uu.conf)][0]
#             v_state = [vv for vv in self.ts.g if np.all(v.conf == vv.conf)][0]
#             self.addTransition(u_state, v_state)
#         print self.ts.g.number_of_nodes()
#         print
