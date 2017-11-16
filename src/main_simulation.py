license_text='''
    Script to setup a planner for a planar unicycle robot.
    Copyright (C) 2016  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab,
    Boston University

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

import sys
import logging
import argparse
import os

import numpy as np

# from firm import SE2BeliefState as BeliefState
from firm.roverbeliefspace import RoverBeliefState as BeliefState
from firm.roverbeliefspace import Map
# from firm import SE2StateSampler as StateSampler
from firm.roverbeliefspace import SE2ShadowStateSampler as StateSampler
from firm import UnicycleMotionModel, CameraLocalization, FIRM, Mission

import matplotlib.pyplot as plt

class FIRM2DSetup(object):
    '''Class to set up a mission for a planar unicycle robot and a copter.'''
    
    def __init__(self, data=None):
        self.setup(data)
    
    def setup(self, mission):
        if mission is None:
            self.isSetup = False
            return
        
        np.random.seed(mission.planning.get('seed', None))
        
        # setting the mean and norm weights (used in reachability check)
        cc = mission.planning['setup']['controller']
        BeliefState.covNormWeight = cc['norm_covariance_weight']
        BeliefState.meanNormWeight = cc['norm_mean_weight']
        BeliefState.reachDist = cc['reach_dist']
        # set the state component norm weights
        BeliefState.normWeights = np.array(cc['norm_state_weights'])
        
        # load environment
        self.env = mission.environment
        # load robot model
        assert mission.no_active_vehicles == 2
        self.robotModel = mission.vehicles.itervalues().next()
        # read the start Pose
        x, y, yaw = self.robotModel['initial_state']
        cov = self.robotModel['initial_covariance']
        # read the specification
        self.spec  = mission.specification
        # read planning time
        self.planningTime = mission.planning['planning_time']
        self.planningSteps = mission.planning['planning_steps']
        self.sat_prob_threshold = mission.planning['sat_probability_threshold']
        # create the observation model
        observationModel = CameraLocalization(self.robotModel, self.env)
        # create the motion model
        motionModel = UnicycleMotionModel(self.robotModel)
        # create planner
        self.planner = FIRM(self.env, motionModel, observationModel,
                            mission.planning['setup'])
        # set specification
        state_label = self.robotModel['motion_model']['state_label']
        cov_label = self.robotModel['motion_model']['covariance_label']
        data = dict()
        execfile(mission.predicates, data)
        predicates = data['predicates']
        layer_priors = data['layer_priors']
        collectables = data['collectables']
        self.planner.obstacles = data['obstacles']
        del data
        assert predicates is not None, \
                            'You need to provide at least an empty dictionary!'
        self.planner.setSpecification(self.spec, state_label=state_label,
                                      cov_label=cov_label,
                                      predicates=predicates)
        # initialize map
        self.prior_map = Map(self.planner.bounds, mission.map_resolution,
                             layer_priors, frozenset(collectables))
#         self.planner.initMap(layer_priors, frozenset(collectables),
#                              step=mission.map_resolution)
        # set initial state
        self.start = BeliefState(x, y, yaw, cov, self.prior_map, freeze=True)
        self.planner.addState(self.start, initial=True)
        # set sampler
        self.planner.sampler = StateSampler(self.planner.bounds
                            + np.array([[0.2, 0.2], [-0.2, -0.2]]),
                            maps = self.prior_map) #TODO: make this general
#         self.planner.sampler = SE2StateSampler(self.planner.bounds
#                 + np.array([[0.2, 0.2], [-0.2, -0.2]])) #TODO: make this general

        logging.info('Problem configuration is')
        logging.info("Start Pose: %s", str(self.start))
        logging.info("Specification: %s", self.spec)
        logging.info("Planning Time: %f seconds", self.planningTime)

        self.isSetup = True


# configure logging
fs, dfs = '%(asctime)s %(levelname)s %(message)s', '%m/%d/%Y %I:%M:%S %p'
loglevel = logging.INFO #logging.DEBUG if args.debug else logging.INFO
logging.basicConfig(filename='data/mars/mars.log', level=loglevel,
                    format=fs, datefmt=dfs)
if True:
    root = logging.getLogger()
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(loglevel)
    ch.setFormatter(logging.Formatter(fs, dfs))
    root.addHandler(ch)

# 1. load mission and environment
mission = Mission.from_file('data/mars/mission.yaml')
logging.info('\n' + str(mission))
logging.info('Seed: %s', mission.planning.get('seed', None))

# 2. setup the systems, and planners
my_setup = FIRM2DSetup()
# setup planner
my_setup.setup(mission)

# 3. execute planners for the rover and copter
fname = os.path.join('data/mars',
                    mission.planning.get('solution_filename', 'sol_{}.txt'))

# HACK: reveal regions of the map
explore = [
    ((0.0, 0.0), (2.8, 1.2)),
    ((0.0, 0.0), (2.8, 3.6)),
    ((0.0, 0.0), (3.8, 3.6)),
    ((0.0, 0.0), (4.8, 3.6)),
]
explore_counter = 0
goal_found = 0
abort_signal = 0

prob_cutoff = 0.5

# TODO: attach abort_signal to callback

while not goal_found:

    # Look for solution
    my_setup.planner.solve(steps=my_setup.planningSteps,
                            maxtime=my_setup.planningTime,
                            epsilon=my_setup.sat_prob_threshold)

    # mission.visualize(my_setup.planner.ts, plot='plot')

    if my_setup.planner.found:

        abort_signal = 0

        # Execute solution
        policy = my_setup.planner.computePolicy()

        # real state
        state = my_setup.start.copy()  # covariance should be disregarded

        state_traj = np.zeros([0,3])
        state_traj = np.vstack([state_traj, state.conf])

        # belief state
        bstate = my_setup.start.copy()

        m_model = my_setup.planner.motion_model
        o_model = my_setup.planner.observation_model

        bstate.freeze()

        while not abort_signal:

            # select target
            target, sat_prob = policy.get_target(bstate)

            if sat_prob < prob_cutoff:
                # probability of satisfaction low: abort!
                abort_signal = True
                break
            
            # move towards target
            print "going to ", target.conf

            controller = my_setup.planner.generateNodeController(target)
            t = 0

            # move towards target
            while not target.isReached(bstate) and not abort_signal:
                # Get control and noise
                u = controller.feedbackController.generateFeedbackControl(state, t)
                w = m_model.generateNoise(state, u)

                # update real state
                state = m_model.evolve(state, u, w)

                state_traj = np.vstack([state_traj, state.conf])

                # Get observation
                z = o_model.getObservation(state)

                # Update bstate
                currentLS = controller.feedbackController.getCurrentLS(t)
                nextLS = controller.feedbackController.getNextLS(t)
                bstate = controller.filter.evolve(bstate, u, z, currentLS, nextLS)

                # APs
                aps = my_setup.planner.getAPs(bstate)
                policy.update_events(aps)

                t += 1

            print "reached ", bstate.conf

            bstate = target   # must do this for policy to work

            target, sat_prob = policy.get_target(bstate)

            if policy.spec_state in my_setup.planner.automaton.final[0][0]:
                goal_found = True
                break

    elif abort_signal:
        # TODO: CLEAR PLANNER AND RESTART FROM SAME SPEC STATE
        print "received abort signal, aborting"
        break
    else:
        # Explore new region
        if explore_counter > len(explore):
            print "explored all regions without finding solution"
            break

        step = my_setup.prior_map.step
        (x_low, y_low), (x_high, y_high) = explore[explore_counter]
        for x in range(int(x_low/step), int((x_high-x_low)/step)):
            for y in range(int(y_low/step), int((y_high-y_low)/step)):
                my_setup.prior_map.layers['shadow'][y][x] = False
        xl, yl = x_high-x_low, y_high-y_low
        mission.environment['regions']['Shadow']['position'] = \
                                                    [x_low+xl/2, y_low+yl/2, 0]
        mission.environment['regions']['Shadow']['sides'] = [xl, yl]

        explore_counter += 1