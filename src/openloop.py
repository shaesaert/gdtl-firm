#!/usr/bin/env python
license_text='''
    Execute open loop controls using the offline computed transition system.
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
'''
Created on Jan 22, 2016

@author: cristi
'''

import os, sys
import logging
import argparse
import itertools as it
import time

import yaml
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
# import rospy

from lomap import Ts, Buchi, ts_times_buchi, source_to_target_dijkstra, Timer
from firm import Mission, UnicycleMotionModel, SE2BeliefState
from firm.utils import normalizeAngle

from robots import ScoutPro, X80Pro
from optitrack import OptiTrackInterface


def ts_load(fname, mission, motionModel):
    ts = Ts(multi=False)
    with open(fname, 'r') as fin:
        data = yaml.load(fin)
    
    data['ts'] = [((pxu, pyu, normalizeAngle(thu)), (pxv, pyv, normalizeAngle(thv)))
                    for (pxu, pyu, thu), (pxv, pyv, thv) in data['ts']]
    
    ts.g.add_nodes_from([u for u, _ in data['ts']] + [v for _, v in data['ts']])
    ts.g.add_edges_from(((x, x) for x in ts.g.nodes_iter()), weight=1)
    ts.init[data['ts_init']] = 1 # add initial state
    assert data['ts_init'] in ts.g # check if initial state is in Ts
    
    # expand edges based on open loop controls, adds states and transitions
    for u, v in data['ts']:
        assert u in ts.g
        assert v in ts.g
        traj, controls = motionModel.generateOpenLoopTrajectory(
                                            SE2BeliefState(u[0], u[1], u[2]),
                                            SE2BeliefState(v[0], v[1], v[2]))
        assert np.allclose(traj[0].conf, u, rtol=0, atol=1e-8)
        assert np.allclose(traj[-1].conf, v, rtol=0, atol=1e-8)
        traj[0].conf[:] = u
        traj[-1].conf[:] = v
        assert tuple(map(float, traj[0].conf)) == u
        assert tuple(map(float, traj[-1].conf)) == v
        
        ts.g.add_edges_from(
        [(tuple(map(float,x.conf)), tuple(map(float, y.conf)),
          {'control': c, 'weight': 1})
            for x, y, c in zip(traj[:-1], traj[1:], controls)])
    
    # label states
    region_data = mission.environment['regions']
    regions = dict([(reg.lower(), (np.array(d['position']), np.array(d['sides'])))
                                        for reg, d in region_data.iteritems()])
    regions.update([(reg, np.array([[c[0] - s[0]/2.0, c[0] + s[0]/2.0],
                                    [c[1] - s[1]/2.0, c[1] + s[1]/2.0]]))
                    for reg, (c, s) in regions.iteritems()])
    inBox = lambda x, b: (b[0, 0] <= x[0] <= b[0, 1]) and (b[1, 0] <= x[1] <= b[1, 1])
     
    for reg, b in regions.iteritems():
        print 'Region:', reg, 'x:', b[0], 'y:', b[1]
    print
     
    print 'TS:', ts.g.number_of_nodes(), ts.g.number_of_edges()
     
    for x in ts.g.nodes_iter():
        ts.g.node[x]['prop'] = set([reg for reg, b in regions.iteritems() if inBox(x, b)])
    
    return ts

def compute_satisfying_run(ts, ltl_spec, loops=2):
    logging.info('Specification: %s', ltl_spec)
    buchi = Buchi()
    buchi.buchi_from_formula(ltl_spec)
    
    with Timer('Product automaton construction'):
        pa = ts_times_buchi(ts, buchi)
        assert pa.init and pa.final
    
    logging.info('PA size: (%d, %d)', pa.g.number_of_nodes(),
                 pa.g.number_of_edges())
    logging.info('PA size init: %d, final: %d', len(pa.init), len(pa.final))
    
    assert len(pa.init) == 1
    pa_init = iter(pa.init).next()
    paths = nx.shortest_path(pa.g, source=pa_init)
    
    traj = None
    for k, final in enumerate(pa.final):
        suffix = source_to_target_dijkstra(pa.g, final, final)
        if not suffix:
            continue
        suffix = suffix[1][1:]
        nsuffix = len(suffix)
        logging.debug('[compute_satisfying_run] Step: %d', k)
        assert suffix, suffix
#         for init in pa.init:
#             prefix = source_to_target_dijkstra(pa.g, init, final)
#             print 'nprefix:', len(prefix)
#             print 'prefix:', prefix
#             if traj is None or len(traj[0])+len(traj[1]) > len(prefix) + nsuffix:
#                 traj = prefix, suffix
        prefix = paths[final]
        if traj is None or len(traj[0])+len(traj[1]) > len(prefix) + nsuffix:
            traj = prefix, suffix
    
    prefix, suffix = traj
    traj = prefix + [p for p in it.islice(it.cycle(suffix), 0, loops*len(suffix))]
    logging.info('Satisfying PA trajectory: %s', traj)
    logging.info('Length of the satisfying policy: %d', len(traj))
    assert prefix[-1] == suffix[-1] and suffix[-1] in pa.final
    return [x for x, _ in traj]

def getTruePose(rid):
    # read sensors -- OptiTrack
    opti = OptiTrackInterface()
    body = opti.trackInfo()[rid] # get data for rigid body w/ rigid body ID `rid'
    opti._close()
    logging.debug('Optitrack raw data: %s', (body.x, body.z, body.yaw, np.deg2rad(body.yaw)))
    return body.z+1.667888, body.x+0.70266, np.deg2rad(body.yaw)+np.pi/2

def executeControl(robot, u, dt=0.25):
    linearVelocity, angularVelocity = u
#     linearVelocity = min(linearVelocity, robot.pulse2vel(1200))
#     angularVelocity = max(min(angularVelocity, robot.pulse2vel(5000)), robot.pulse2vel(-5000))
    
    print robot.pulse2vel(900), robot.pulse2vel(5000)
    
    logging.info('Lin Velocity: %f %f', linearVelocity, robot.vel2pulse(linearVelocity))
    logging.info('Ang Velocity: %f %f', angularVelocity, robot.vel2pulse(angularVelocity))
    # 3. send commands to the robot
#     linearVelocity = 1000 #TODO: remove after test
#     angularVelocity = 0
    robot.setVelocity(linearVelocity, angularVelocity)
    time.sleep(dt)
    

def executeController(args, controls, desired_trajectory):
    # create robot object
    if args.robot == 'X80Pro1':
        robot = X80Pro('DrRobot1', '192.168.0.202', 10001) 
    elif args.robot == 'X80Pro2':
        robot = X80Pro('DrRobot1', '192.168.0.201', 10001)
    elif args.robot == 'ScoutPro1':
        robot = ScoutPro('DrRobot1', '192.168.0.220', 10002)
    elif args.robot == 'ScoutPro2':
        robot = ScoutPro('DrRobot1', '192.168.0.218', 10002)
    else:
        raise ValueError('Unknown robot!')
    
    real_trajectory = []
    try:
        for k, u in enumerate(controls):
#         for k, u in enumerate(it.repeat([0.2, 0], times=4*3)):
            z = getTruePose(args.id)
            logging.info('Step %d: true pose: %s', k, z)
            real_trajectory.append(z)
            executeControl(robot, u)
    finally:
        robot.setSpeed([0, 0])
    
    # 5. plot
    if args.plot:
        alen = 0.05
        
        # 1. load mission and environment
        mission = Mission.from_file(args.mission_file)
        
        region_data = mission.environment['regions']
        regions = dict([(reg.lower(), (np.array(d['position']), np.array(d['sides'])))
                                            for reg, d in region_data.iteritems()])
        regions.update([(reg, np.array([[c[0] - s[0]/2.0, c[0] + s[0]/2.0],
                                        [c[1] - s[1]/2.0, c[1] + s[1]/2.0]]))
                        for reg, (c, s) in regions.iteritems()])
        region_colors = dict([(reg.lower(), d['color'])
                               for reg, d in region_data.iteritems()])
        figure = plt.figure()
        plt.axis('equal') 
        for reg, b in regions.iteritems():
            x, y = b[0, 0], b[1, 0]
            l, w = b[0, 1]-x, b[1, 1]-y
            r = plt.Rectangle((x, y), l, w, color=region_colors[reg], fill=True,
                              linewidth=2)
            figure.gca().add_artist(r)
            plt.text(np.sum(b[0])/2, np.sum(b[1])/2, reg)
        
        for x, y, yaw in desired_trajectory:
            plt.arrow(x, y, alen*np.cos(yaw), alen*np.sin(yaw), hold=True, color='green')
        for x, y, yaw in real_trajectory:
            plt.arrow(x, y, 0.1*np.cos(yaw), 0.1*np.sin(yaw), hold=True, color='k')
        xi, yi, yawi = desired_trajectory[0]
        plt.arrow(xi, yi, alen*np.cos(yawi), alen*np.sin(yawi), hold=True, color='blue')
        plt.xlim([0, 4.13])
        plt.ylim([0, 3.54])
        plt.show()

def run():
    # 0. parse command-line arguments
    parser = argparse.ArgumentParser(
        description=
'''Script to execute an open loop trajectory on the robot. The open loop
trajectory is generated using the offline computed transition system where the
states are projected on the workspace. Also, the specification considered for
synthesis does not include constraints on the uncertainty and must be provided
the user as an LTL formula. It is the responsibility of the user to ensure that
the simplified specification is semantically close to the original GDTL formula.
''',
        epilog=license_text)
    
    parser.add_argument('mission_file', type=str, help='mission file')
    parser.add_argument('ts_file', type=str, help='transition system file')
    parser.add_argument('ltl_spec', type=str,
                        help='LTL (simplified) specification')
    parser.add_argument('-o', '--output-dir', type=str, help='output directory')
    parser.add_argument('-l', '--logfile', type=str, help='log file',
                        default='openloop.log')
    parser.add_argument('-p', '--plot', action='store_true',
                        help='plot ground truth data')
    parser.add_argument('-r', '--robot', action='store', default='ScoutPro1',
                        type=str,
                        help='select the robot to use (default: ScoutPro1)',
                        choices=['X80Pro1', 'X80Pro2', 'ScoutPro1', 'ScoutPro2'])
    parser.add_argument('-i', '--id', action='store', default=1, type=int,
                        help='select the rigid body id for OptiTrack associated with the robot (default: 1)')
    parser.add_argument('--debug', action='store_true', help='debug mode')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='print/plot extra information for simulation')
    # parse arguments
    args = parser.parse_args()
    # create output directory
    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir)
    # configure logging
    fs, dfs = '%(asctime)s %(levelname)s %(message)s', '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(filename=args.logfile, level=loglevel, format=fs, datefmt=dfs)
    if args.verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)
    
    logging.info('Script arguments: %s, %s', args.robot, args.id)
    
    fcontrols = os.path.join(args.output_dir, 'openloop_controls.txt')
    ftraj = os.path.join(args.output_dir, 'openloop_traj.txt')
    if os.path.isfile(fcontrols) and os.path.isfile(ftraj):
        controls = np.loadtxt(fcontrols, dtype=np.float, delimiter='\t')
        traj = np.loadtxt(ftraj, dtype=np.float, delimiter='\t')
    else:
        # 1. load mission and environment
        mission = Mission.from_file(args.mission_file)
        
        # 2. load transition system
        assert mission.no_active_vehicles == 1
        robotModel = mission.vehicles.itervalues().next()
        
        motionModel = UnicycleMotionModel(robotModel, isSimulation=True)
    #     motionModel = OmnidirectionalMotionModel(robotModel, isSimulation=False)
        ts = ts_load(args.ts_file, mission, motionModel)
        
        # 3. compute satisfying trajectory with respect to the LTL spec
        with open(args.ltl_spec, 'r') as fin:
            ltl_spec = fin.readline().strip()
        traj = compute_satisfying_run(ts, ltl_spec)
        
        # 4. execute satisfying trajectory
        controls = [ts.g[s][t]['control'] for s, t in it.izip(traj[:-1], traj[1:])]
        logging.info('Length of control sequence: %d', len(controls))
        
        # 5. save trajectory and control policy
        np.savetxt(ftraj, traj, delimiter='\t', header=time.asctime())
        np.savetxt(fcontrols, controls, delimiter='\t', header=time.asctime())
    
    executeController(args, controls, traj)
    
    logging.info('Done!')

if __name__ == '__main__':
    run()
