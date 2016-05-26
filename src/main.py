license_text='''
    Script to setup a planner for a planar unicycle robot.
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

import sys
import logging
import argparse
import os

import numpy as np

from firm import SE2BeliefState, SE2StateSampler, \
                 UnicycleMotionModel, \
                 CameraLocalization, \
                 FIRM, Mission


class FIRM2DSetup(object):
    '''Class to set up a mission for a planar unicycle robot.'''
    
    def __init__(self, data=None):
        self.setup(data)
    
    def setup(self, mission):
        if mission is None:
            self.isSetup = False
            return
        
        np.random.seed(mission.planning.get('seed', None))
        
        # setting the mean and norm weights (used in reachability check)
        cc = mission.planning['setup']['controller']
        SE2BeliefState.covNormWeight = cc['norm_covariance_weight']
        SE2BeliefState.meanNormWeight = cc['norm_mean_weight']
        SE2BeliefState.reachDist = cc['reach_dist']
        # set the state component norm weights
        SE2BeliefState.normWeights = np.array(cc['norm_state_weights'])
        
        # load environment
        self.env = mission.environment
        # load robot model
        assert mission.no_active_vehicles == 1
        self.robotModel = mission.vehicles.itervalues().next()
        # read the start Pose
        x, y, yaw = self.robotModel['initial_state']
        cov = self.robotModel['initial_covariance']
        self.start = SE2BeliefState(x, y, yaw, cov)
        # read the specification
        self.spec  = mission.specification
        # read planning time
        self.planningTime = mission.planning['planning_time']
        self.planningSteps = mission.planning['planning_steps']
        self.sat_prob_threshold = mission.planning['sat_probability_threshold']
        
        logging.info('Problem configuration is')
        logging.info("Start Pose: %s", str(self.start))
        logging.info("Specification: %s", self.spec)
        logging.info("Planning Time: %f seconds", self.planningTime)
        
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
        del data
        assert predicates is not None, \
                            'You need to provide at least an empty dictionary!'
        self.planner.setSpecification(self.spec, state_label=state_label,
                                      cov_label=cov_label,
                                      predicates=predicates)
        # set initial state
        self.planner.addState(self.start, initial=True)
        # set sampler
        self.planner.sampler = SE2StateSampler(self.planner.bounds + np.array([[0.2, 0.2], [-0.2, -0.2]])) #TODO: make this general
    
    def solve(self, load=None, save=None):
        if load:
            self.planner.loadRoadMapFromFile(load)
        r = self.planner.solve(steps=self.planningSteps,
                               maxtime=self.planningTime,
                               epsilon=self.sat_prob_threshold)
        if save:
            self.planner.savePlannerData(save)
        return r
    
    def execute_solution(self):
        self.planner.executePolicy()

def plan():
    # 0. parse command-line arguments
    parser = argparse.ArgumentParser(
        description='''Sampling-based mission planner with Distribution Temporal
Logic specifications.''',
        epilog=license_text)
    
    parser.add_argument('mission_file', type=str, help='mission file')
    parser.add_argument('-o', '--output-dir', type=str, help='output directory')
    parser.add_argument('-l', '--logfile', type=str, help='log file',
                        default='gdtlfirm.log')
    parser.add_argument('-d', '--design', action='store_true',
                        help='only show environment')
    parser.add_argument('-p', '--plot', choices=['figures', 'video', 'none'],
                        help='plot simulation data', default='none')
    parser.add_argument('--debug', action='store_true',
                        help='debug mode')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='print/plot extra information for simulation')
    # parse arguments
    args = parser.parse_args()
    # create output directory
    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir)
    # configure logging
    fs, dfs = '%(asctime)s %(levelname)s %(message)s', '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.INFO #logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(filename=args.logfile, level=loglevel, format=fs, datefmt=dfs)
    if args.verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)
    
    # 1. load mission and environment
    mission = Mission.from_file(args.mission_file)
    logging.info('\n' + str(mission))
    logging.info('Seed: %s', mission.planning.get('seed', None))
    
    if args.design: # only shows the environment
        mission.visualize(None, plot='design')
        return
    
    # 2. setup, find solution and execute it
    my_setup = FIRM2DSetup()
    # setup planner
    my_setup.setup(mission)
    # solve problem and if successful execute it
    fname = os.path.join(args.output_dir,
                               mission.planning.get('solution_filename', None))
    
#     if my_setup.solve(save=fname):
#         my_setup.execute_solution()
#         logging.info('Plan Executed Successfully.')
#     else:
#         logging.info('Unable to find Solution in given time.')
#     logging.info('Task Complete')
    
    mission.visualize(my_setup.planner.ts, plot='plot')

if __name__ == '__main__':
    plan()
