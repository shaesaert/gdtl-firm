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

explore_counter = 0     # index of region to explore next
goal_found = False      # boolean

planning_counter = 1    # num of times to plan before exploring

prob_cutoff = 0.5       # min acceptable probability of satisfaction

nstep = 0               # 

next_option = 'solve'   # 'solve', 'extend', 'replan', 'execute'

state_traj = np.zeros([0,3])

# TODO: attach abort_signal to callback

while not goal_found:

    if next_option == 'solve':

        # Look for solution
        print "extending ts.."
        my_setup.planner.solve(steps=my_setup.planningSteps,
                                maxtime=my_setup.planningTime,
                                epsilon=my_setup.sat_prob_threshold)
        planning_counter -= 1
        mission.visualize(my_setup.planner.ts, plot='plot')

        if my_setup.planner.found:
            next_option = 'execute'

        elif planning_counter <= 0:
            next_option = 'explore'
            planning_counter = 1

    if next_option == 'execute':
        print "executing.."

        # Execute solution
        policy = my_setup.planner.computePolicy()

        # real state
        state = my_setup.start.copy()  # covariance should be disregarded

        state_traj = np.vstack([state_traj, state.conf])

        # belief state
        bstate = my_setup.start.copy()

        m_model = my_setup.planner.motion_model
        o_model = my_setup.planner.observation_model

        bstate.freeze()

        while next_option == 'execute':

            # select target
            target, sat_prob = policy.get_target(bstate)

            if sat_prob < prob_cutoff:
                # probability of satisfaction low: abort!
                next_option = 'replan'
                break
            
            # move towards target
            print "at ", bstate.conf, ", going to ", target.conf

            controller = my_setup.planner.generateNodeController(target)
            t = 0

            # move towards target
            while not target.isReached(bstate) and next_option == 'execute':
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

                nstep += 1
                # Manual triggering of replanning!
                if nstep > 100:
                    next_option = 'replan'
                    nstep = 0

                t += 1

            print "reached ", bstate.conf

            bstate = target   # must do this for policy to work

            target, sat_prob = policy.get_target(bstate)

            if policy.spec_state in my_setup.planner.automaton.final[0][0]:
                goal_found = True
                break

    if next_option == 'replan':
        # CLEAR PLANNER AND RESTART FROM SAME SPEC STATE
        print "replanning..."
        
        # Set current initial condition
        my_setup.robotModel['initial_state'] = bstate.conf

        # Restart planner
        my_setup.setup(mission)

        # Set spec (HACK)
        data = dict()
        execfile(mission.predicates, data)
        state_label = my_setup.robotModel['motion_model']['state_label']
        cov_label = my_setup.robotModel['motion_model']['covariance_label']
        predicates = data['predicates']

        my_setup.spec = 'F (A(x, m) > 0) && ((Obs(x) < 1) U ((A(x, m) > 0) && (Obs(x) < 1)))'
        my_setup.planner.setSpecification(my_setup.spec, state_label=state_label,
                                           cov_label=cov_label,
                                           predicates=predicates)

        # Uncover map (again)
        step = my_setup.prior_map.step
        for i in range(explore_counter):
            (x_low, y_low), (x_high, y_high) = explore[i]
            for x in range(int(x_low/step), int((x_high-x_low)/step)):
                for y in range(int(y_low/step), int((y_high-y_low)/step)):
                    my_setup.prior_map.layers['shadow'][y][x] = False


        planning_counter = 3
        next_option = 'solve'

    if next_option == 'explore':
        # Explore new region
        if explore_counter > len(explore):
            print "explored all regions without finding solution"
            break

        print "exploring new region.."
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
        next_option = 'solve'

print "made it to goal.."

figure = plt.figure()
figure.add_subplot('111')
axes = figure.axes[0]

mission.draw_environment(axes, figure)
mission.draw_ts(my_setup.planner.ts, axes, figure)
axes.plot(state_traj[:,0], state_traj[:,1])

plt.show()