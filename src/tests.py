#!/usr/bin/env python
'''
Created on Jan 21, 2016

@author: cristi
'''
import sys
import logging
import itertools as it

import matplotlib.pyplot as plt
import numpy as np
import rospy

from firm import Mission, CameraLocalization, OptitrackLocalization, QuadCamPose,\
                 SE2BeliefState, E2BeliefState,\
                 OmnidirectionalMotionModel, UnicycleMotionModel, \
                 LinearizedKF, ExtendedKF, \
                 SLQRController, TrackingLQRController, Controller, \
                 OffAxisSLQRController, SwitchingController


def test_observation_model():
    
    isSimulation = True
    
    if not isSimulation:
        rospy.init_node("ObservationModelTest", anonymous=True)
    
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    logging.info('\n' + str(mission))
    logging.info('Seed: %s', mission.simulation.get('seed', None))
    # load environment
    env = mission.environment
    # load robot model
    assert mission.no_active_vehicles == 1
    robotModel = mission.vehicles.itervalues().next()
    print robotModel['observation_model']
    
    cam = CameraLocalization(robotModel, env, isSimulation=isSimulation)
    
    if not isSimulation:
        rospy.spin()

def test_unicycle_model():
    isSimulation = True
    
    if not isSimulation:
        rospy.init_node("MotionModelTest", anonymous=True)
    
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    logging.info('\n' + str(mission))
    logging.info('Seed: %s', mission.simulation.get('seed', None))
    
    # load environment
    env = mission.environment
    # load robot model
    assert mission.no_active_vehicles == 1
    robotModel = mission.vehicles.itervalues().next()
    print 'Motion model:', robotModel['motion_model']
    
    robot = UnicycleMotionModel(robotModel)
    print 'Time step:', robot.dt
    
    state = SE2BeliefState(1, 2, np.pi/4)
    control = np.array([200, 100])
    
    print 'State Jacobian:'
    print robot.getStateJacobian(state, control)
    print 'Control Jacobian:'
    print robot.getControlJacobian(state, control)
    print 'Noise Jacobian:'
    print robot.getNoiseJacobian(state, control)
    print 'Noise Covariance Matrix'
    print robot.processNoiseCovariance(state, control)
    
    start = SE2BeliefState(0, 0, 0)
    goal = SE2BeliefState(1, 1, np.pi/2)
    
    print 'Open loop path:'
    policy = robot.generateOpenLoopControls(start, goal)
    aux_state = start.copy()
    trajectory = [start]
    print 'Start:', aux_state.getXYYaw()
    for u in policy:
        aux_state = robot.evolve(aux_state, u)
        print 'Control:', u, 'State:', aux_state.getXYYaw()
        trajectory.append(aux_state)
        
    print len(policy)
    
    print 'Generate sample path from open loop policy'
    aux_state = start.copy()
    sample_path = [aux_state] # without initial uncertainty
    for u in policy:
        w = robot.generateNoise(aux_state, u)
        aux_state = robot.evolve(aux_state, u, w)
        print 'Control:', u, 'Noise:', w, 'State:', aux_state
        sample_path.append(aux_state)
    
    plt.figure()
    for s, sn in zip(trajectory, sample_path):
        plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='k')
        plt.arrow(sn.x, sn.y, 0.1*np.cos(sn.yaw), 0.1*np.sin(sn.yaw), hold=True, color='b')
    
    plt.xlim([-0.3, 1.3])
    plt.ylim([-0.3, 1.3])
    plt.show()
    
    print 
    
    if not isSimulation:
        rospy.spin()


def test_omnidirectional_model():
    isSimulation = True
    
    if not isSimulation:
        rospy.init_node("MotionModelTest", anonymous=True)

    
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    logging.info('\n' + str(mission))
    logging.info('Seed: %s', mission.simulation.get('seed', None))
    
    # load environment
#     env = mission.environment
    # load robot model
    assert mission.no_active_vehicles == 1
    robotModel = mission.vehicles.itervalues().next()
    print 'Motion model:', robotModel['motion_model']
    
    robot = OmnidirectionalMotionModel(robotModel, isSimulation)
    print 'Time step:', robot.dt
    
    state = SE2BeliefState(1, 2, np.pi/4)
    control = np.array([200, 200, 100])
    
    print 'State Jacobian:'
    print robot.getStateJacobian(state, control)
    print 'Control Jacobian:'
    print robot.getControlJacobian(state, control)
    print 'Noise Jacobian:'
    print robot.getNoiseJacobian(state, control)
    print 'Noise Covariance Matrix'
    print robot.processNoiseCovariance(state, control)
    
    start = SE2BeliefState(0, 0, 0)
    goal = SE2BeliefState(1, 1, np.pi/2)
    
    print 'Open loop path:'
    policy = robot.generateOpenLoopControls(start, goal)
    aux_state = start
    trajectory = [aux_state.copy()]
    print 'Start:', aux_state.getXYYaw()
    for u in policy:
        aux_state = robot.evolve(aux_state, u)
        print 'Control:', u, 'State:', aux_state.getXYYaw()
        trajectory.append(aux_state)
        
    print len(policy)
    
    print 'Generate sample path from open loop policy'
    aux_state = start.copy()
    sample_path = [aux_state.copy()] # without initial uncertainty
    for u in policy:
        w = robot.generateNoise(aux_state, u)
        aux_state = robot.evolve(aux_state, u, w)
        print 'Control:', u, 'Noise:', w, 'State:', aux_state.getXYYaw()
        sample_path.append(aux_state)
    
    plt.figure()
    for s, sn in zip(trajectory, sample_path):
        plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='k')
        plt.arrow(sn.x, sn.y, 0.1*np.cos(sn.yaw), 0.1*np.sin(sn.yaw), hold=True, color='b')
    
    plt.xlim([-0.3, 1.3])
    plt.ylim([-0.3, 1.3])
    plt.show()
    
    print 
    
    if not isSimulation:
        rospy.spin()

# def test_controller():
#     isSimulation = True
#     
#     if not isSimulation:
#         rospy.init_node("ControllerTest", anonymous=True)
#     
#     # 1. load mission and environment
#     mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
#     logging.info('\n' + str(mission))
#     logging.info('Seed: %s', mission.simulation.get('seed', None))
#     
#     # load environment
#     env = mission.environment
#     # load robot model
#     assert mission.no_active_vehicles == 1
#     robotModel = mission.vehicles.itervalues().next()
#     print 'Motion model:', robotModel['motion_model']
#     
#     cc = mission.planning['setup']['controller']
#     SE2BeliefState.covNormWeight  =  cc['norm_covariance_weight']
#     SE2BeliefState.meanNormWeight =  cc['norm_mean_weight']
#     SE2BeliefState.reachDist =  cc['reach_dist']
#     # set the state component norm weights
#     SE2BeliefState.normWeights = np.array(cc['norm_state_weights'])
#     
#     robot = UnicycleMotionModel(robotModel, isSimulation)
#     print 'Time step:', robot.dt
#     
#     print 'Observation model:', robotModel['observation_model']
#     cam = CameraLocalization(robotModel, env, isSimulation=isSimulation)
#     
#     def save_trajectory():
#         true_traj = []
#         traj = []
#         while True:
#             state = yield traj, true_traj
#             print 'save state:', state.conf
#             traj.append(state.copy())
#             true_traj.append(cam.getObservation(state))
#     
#     start = SE2BeliefState(-0.38, 1.47, -0.05)
#     goal = SE2BeliefState(1-0.38, 2.47, np.pi/2) # NOTE: end orientation does not matter
#     
#     tlqr = TrackingLQRController(robot, cam, (start, goal),
#                                  stateWeight=np.diag([1, 1, 200]), controlWeight=np.eye(2))
#     ekf = ExtendedKF(cam, robot)
#     controller = Controller(robot, cam, tlqr, ekf, withNoise=False)
#     
#     process = save_trajectory()
#     process.next()
#     _, (trajectory, true_trajectory) = controller.execute(start, process)
#     robot.execute([0, 0])
#     print len(trajectory)
#     
#     plt.figure()
#     plt.axis('equal')
#     for lss in tlqr.lss:
#         s = lss.x
#         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='g')
#     for x, y, yaw in true_trajectory:
#         plt.arrow(x, y, 0.1*np.cos(yaw), 0.1*np.sin(yaw), hold=True, color='k')
#     for s in trajectory:
#         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='b')
#     
# #     plt.xlim([0, 4.13])
# #     plt.ylim([0, 3.54])
#     plt.xlim([-0.7, 1.2])
#     plt.ylim([1.1, 2.7])
#     plt.show()
#     
#     print


def test_controller__():
    isSimulation = False
    
    if not isSimulation:
        rospy.init_node("ControllerTest", anonymous=True)
    
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    logging.info('\n' + str(mission))
    logging.info('Seed: %s', mission.simulation.get('seed', None))
    
    # load environment
    env = mission.environment
    # load robot model
    assert mission.no_active_vehicles == 1
    robotModel = mission.vehicles.itervalues().next()
    print 'Motion model:', robotModel['motion_model']
    
    cc = mission.planning['setup']['controller']
    SE2BeliefState.covNormWeight  = cc['norm_covariance_weight']
    SE2BeliefState.meanNormWeight = cc['norm_mean_weight']
    SE2BeliefState.reachDist = cc['reach_dist']
    # set the state component norm weights
    SE2BeliefState.normWeights = np.array(cc['norm_state_weights'])
    SE2BeliefState.normWeights[2] = 0
    
    robot = UnicycleMotionModel(robotModel, isSimulation)
    print 'Time step:', robot.dt
    
#     from robots.X80Pro import X80Pro
#     import time
#     robot.rate.sleep()
#      
#     u = 0.25
#     t0 = time.time()
#     print 'Applying control', t0, 'control in pulse:', X80Pro.vel2pulse(u)
# #     robot.execute([0, 6.5]) # TODO: rotation ->  np.pi
#     robot.execute([u, 0]) # linear
#     print 'Control sent', time.time() - t0
#     time.sleep(4-0.25)
#     print 'Stop robot', time.time() - t0
#     robot.execute([0, 0])
#     print 'Done', time.time() - t0
#     return
    
    print 'Observation model:', robotModel['observation_model']
    cam = CameraLocalization(robotModel, env, isSimulation=isSimulation)
    
    def save_trajectory():
        true_traj = []
        traj = []
        while True:
            state = yield traj, true_traj
            print 'save state:', state.conf
            traj.append(state.copy())
            true_traj.append(cam.getObservation(state)) #TODO: change this back
#             true_traj.append(cam.getObservationPrediction(state))
    
    start = SE2BeliefState(0.240, 1.070, 0)
    goal = SE2BeliefState(0.240, 1 + 1.070, np.pi/2) # NOTE: end orientation does not matter
    
    tlqr = SwitchingController(robot, cam, goal,
                               stateWeight=np.diag([1, 1, 200]),
                               controlWeight=np.eye(2))
    ekf = ExtendedKF(cam, robot)
    controller = Controller(robot, cam, tlqr, ekf, withNoise=True) #FIXME: withNoise option is not ok
    
    process = save_trajectory()
    process.next()
    _, (trajectory, true_trajectory) = controller.execute(start, process)
    robot.execute([0, 0])
    print len(trajectory)
    
#     plt.figure()
#     plt.axis('equal')
# #     for lss in tlqr.lss:
# #         s = lss.x
# #         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='g')
#     for x, y, yaw in true_trajectory:
#         plt.arrow(x, y, 0.1*np.cos(yaw), 0.1*np.sin(yaw), hold=True, color='k')
#     for s in trajectory:
#         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='b')
#     s = goal
#     plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='g')
#     
# #     plt.xlim([0, 4.13])
# #     plt.ylim([0, 3.54])
#     plt.xlim([-0.7, 1.2])
#     plt.ylim([1.1, 2.7])
#     plt.show()
    
    print

#     slqr = OffAxisSLQRController(robot, cam, goal,
#                                  stateWeight=np.diag([1, 1, 200]),
#                                  controlWeight=np.eye(2))
#     lkf = LinearizedKF(cam, robot)
#     controller = Controller(robot, cam, slqr, lkf, withNoise=False)
#     
#     process = save_trajectory()
#     process.next()
#     _, (trajectory, true_trajectory) = controller.execute(trajectory[-1], process)
#     print len(trajectory)
#     
#     plt.figure()
# #     for lss in tlqr.lss:
# #         s = lss.x
# #         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='g')
#     for x, y, yaw in true_trajectory:
#         plt.arrow(x, y, 0.1*np.cos(yaw), 0.1*np.sin(yaw), hold=True, color='k')
#     for s in trajectory:
#         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='b')
#     s = goal
#     plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='g')
#     s = trajectory[-1]
#     plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='r')
#     
#     plt.xlim([-0.7, 1.2])
#     plt.ylim([1.1, 2.7])
#     plt.show()
#     
#     print

# def test_controller_offaxis():
#     isSimulation = True
#     
#     # 1. load mission and environment
#     mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
#     logging.info('\n' + str(mission))
#     logging.info('Seed: %s', mission.simulation.get('seed', None))
#     
#     # load environment
#     env = mission.environment
#     # load robot model
#     assert mission.no_active_vehicles == 1
#     robotModel = mission.vehicles.itervalues().next()
#     print 'Motion model:', robotModel['motion_model']
#     
#     cc = mission.planning['setup']['controller']
#     SE2BeliefState.covNormWeight  =  cc['norm_covariance_weight']
#     SE2BeliefState.meanNormWeight =  cc['norm_mean_weight']
#     SE2BeliefState.reachDist =  cc['reach_dist']
# #         # set the state component norm weights
#     SE2BeliefState.normWeights = np.array(cc['norm_state_weights'])
#     
#     robot = UnicycleMotionModel(robotModel, isSimulation)
#     print 'Time step:', robot.dt
#     
#     print 'Observation model:', robotModel['observation_model']
#     cam = CameraLocalization(robotModel, env, isSimulation=isSimulation)
#     
#     start = SE2BeliefState(0, 0, 0)
#     goal = SE2BeliefState(1, 1, np.pi/2)
#     
#     slqr = OffAxisSLQRController(robot, cam, goal, np.array(cc['state_weight']),
#                                  np.array(cc['control_weight']))
# #     slqr = SLQRController(robot, cam, goal, np.eye(3), np.eye(3))
#     lkf = LinearizedKF(cam, robot)
#     controller = Controller(robot, cam, slqr, lkf, withNoise=False)
# #      
#     def save_trajectory():
#         traj = []
#         while True:
#             state = yield traj
#             print 'save state:', state.conf
#             traj.append(state.copy())
#       
#     process = save_trajectory()
#     process.next()
#     _, trajectory = controller.execute(start, process)
#     print len(trajectory)
#       
#     plt.figure()
#     for s in trajectory:
#         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='k')
#     plt.xlim([-0.3, 1.3])
#     plt.ylim([-0.3, 1.3])
#     plt.show()
#        
#     print
#     
#     
#     tlqr = TrackingLQRController(robot, cam, (start, goal),
#                                  stateWeight=np.array(cc['state_weight']),
#                                  controlWeight=np.array(cc['control_weight']))
#     ekf = ExtendedKF(cam, robot)
#     controller = Controller(robot, cam, tlqr, ekf)
#      
#     process = save_trajectory()
#     process.next()
#     _, trajectory = controller.execute(start, process)
#     print len(trajectory)
#     
#     controller.withNoise = False
#     process = save_trajectory()
#     process.next()
#     _, trajectory_grtruth = controller.execute(start, process)
#     print len(trajectory_grtruth)
#      
#     plt.figure()
#     for s in trajectory:
#         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='k')
#     for s in trajectory_grtruth:
#         plt.arrow(s.x, s.y, 0.1*np.cos(s.yaw), 0.1*np.sin(s.yaw), hold=True, color='b')
#     plt.xlim([-0.3, 1.3])
#     plt.ylim([-0.3, 1.3])
#     plt.show()
#      
#     print

# def test_off_axis_point():
#     L = np.array([[ 0.88278222,  0.        ],
#                   [ 0.        ,  0.88278222]])
#     dt = 0.0025 # [s]
#     d = 0.1 # [m]
#     
#     start = np.array([0.1, 0  ])
#     center_start =  np.array([0.0, 0.0, 0.0])
#     goal  = np.array([1  , 1.1])
#     center_goal =  np.array([1, 1, np.pi/2])
#     
#     traj = []
#     center_traj = []
#     state = np.array(start)
#     center_state = np.array(center_start)
#     for k in it.count():
#         print 'Step', k
# #         assert np.allclose(center_state[0] + d*np.cos(center_state[2]), state[0], rtol=0, atol=1e-4)
# #         assert np.allclose(center_state[1] + d*np.sin(center_state[2]), state[1], rtol=0, atol=1e-4)
# 
#         dist = np.linalg.norm(goal - state)
#         center_dist = np.linalg.norm(center_goal[:2] - center_state[:2])
#         if center_dist < 0.05:
#             print 'reached goal:', dist
#             break
#         traj.append(np.array(state))
#         center_traj.append(np.array(center_state))
#         v = - L.dot(state - goal)
#         print 'Distance:', dist, 'Current state:', state, 'Control:', v
#         cs, ss = np.cos(center_state[2]), np.sin(center_state[2])
#         T = np.array([[cs, ss], [-ss/d, cs/d]])
#         u = T.dot(v)
#         print 'Center distance:', center_dist, 'Current state:', center_state, 'Control:', u
#         state += dt *v
#         center_state += dt * (np.array([u[0]*cs, u[0]*ss, u[1]]))
# #         assert np.allclose(center_state[0] + d*np.cos(center_state[2]), state[0], rtol=0, atol=1e-4)
# #         assert np.allclose(center_state[1] + d*np.sin(center_state[2]), state[1], rtol=0, atol=1e-4)
#         print 'Next state:', state
#         print 'Center next state:', center_state
#         print
#     
# #     plt.plot([x for x, _ in traj], [y for _, y in traj], 'ko')
#     plt.plot([x for x, _, _ in center_traj], [y for _, y, _ in center_traj], 'o', color='blue')
#     plt.show()
#     
#     print 'Done'
#     
# samples = iter([
#        [ 1.16713616,  1.51407632],
#        [ 0.37331931,  0.90593586],
#        [ 1.45708584,  0.79027675],
#        [ 2.09699101,  1.8819757 ],
#        [ 2.41176917,  0.49268686],
#        [ 1.80089743,  3.09878486],
#        [ 3.38058154,  0.24493176],
#        [ 2.76214065,  3.37445037],
#        [ 0.89778079,  3.51157219],
#        [ 2.7637116,   2.45336069],
#        [ 3.29847093,  1.60836811],
#        [ 0.25386715,  3.21167935],
#        [ 0.4618557,  1.5373752],
#        [ 3.68950462,  2.3679684 ],
#        [ 4.01031138,  3.31510251],
#        [ 0.4248697,  0.2581066],
#        [ 3.73767025,  0.75450537],
#        [ 0.35272534,  2.5640234 ],
#        [ 2.52515898,  1.32099854],
#        [ 1.18376672,  0.22991626],
#        [ 4.03114727,  1.79325184],
#        [ 3.21213082,  2.93275297],
#        [ 1.84307696,  0.19041689],
#        [ 1.58607394,  2.36448353]])

def test_controller():
#     from optitrack import OptiTrackInterface
    from numpy.linalg import norm
    import time, datetime
    
    isSimulation = False
    
    if not isSimulation:
        rospy.init_node("ControllerTest", anonymous=True)
    
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    logging.info('\n' + str(mission))
    logging.info('Seed: %s', mission.simulation.get('seed', None))
     
    # load environment
    env = mission.environment
    # load robot model
    assert mission.no_active_vehicles == 1
    robotModel = mission.vehicles.itervalues().next()
    print 'Motion model:', robotModel['motion_model']
     
    cc = mission.planning['setup']['controller']
    SE2BeliefState.covNormWeight  = 0*cc['norm_covariance_weight']
    SE2BeliefState.meanNormWeight = 1 #cc['norm_mean_weight']
    SE2BeliefState.reachDist = cc['reach_dist']
    # set the state component norm weights
    SE2BeliefState.normWeights = np.array([1, 1, 0]) #cc['norm_state_weights'])
    SE2BeliefState.normWeights[2] = 0
     
    robot = UnicycleMotionModel(robotModel, isSimulation)
    print 'Time step:', robot.dt
    robot.rate.sleep()
    
    print 'Observation model:', robotModel['observation_model']
    cam = CameraLocalization(robotModel, env, isSimulation=isSimulation)
    opti = OptitrackLocalization(isSimulation=isSimulation)
    quad = QuadCamPose(isSimulation=isSimulation)
    robot.rate.sleep()
    
    #---------------------------------------------------------------------------
    d_epsilon=0.05 # error bound around waypoint
    a_epsilon = np.deg2rad(15) # error bound for turning
    
#     rid = 1
#     opti = OptiTrackInterface()
#     body = opti.trackInfo()[rid] # get data for rigid body w/ rigid body ID `rid'
#     opti._close()
    x, y, theta = cam.getObservation(None) #body.z, body.x, np.deg2rad(body.yaw)
    print x, y, theta
    #---------------------------------------------------------------------------
    
    def save_trajectory():
        true_traj = []
        traj = []
        optitrack_traj = []
        quad_traj = []
        while True:
            state = yield traj, true_traj, optitrack_traj, quad_traj
            print 'save state:', state.conf
            traj.append(state.copy())
            true_traj.append(cam.getObservation(state)) #TODO: change this back
            optitrack_traj.append(opti.getObservation(state))
            quad_traj.append(quad.getObservation(state))
#             true_traj.append(cam.getObservationPrediction(state))
    
    filter_time = 5 # [sec]
    filter_stab  = int(filter_time/robot.dt)
    print 'filter steps:', filter_stab, 'dt:', robot.dt
    
    xs, ys = x, y
    covs = np.eye(3)
    
#     path = np.array([ # larger transition system
#         [1.0, 2.5, 0.0],
#         [1.227090539137495, 1.5261366178286393, 0],
#         [0.7814101593838313, 0.6309556309870786, 0],
#         [1.3450599380271273, 0.880746156908613, 0],
#         [2.063849180607642, 0.9785829893820421, 0],
#         [2.8594440505001724, 1.354833717086771, 0],
#         [3.187859804988132, 1.976122679525189, 0],
#         [3.6840077044823207, 2.8443491833405843, 0],
#         [2.925239900621002, 2.52353894593588, 0],
#         [2.288019681537405, 2.549151027045657, 0],
#         [1.9702269162937254, 3.102135677048742, 0],
#         [1.306980698970712, 3.091532230704365, 0],
#         [1.0, 2.5, 0.0]
#     ])
    
    path = np.array([ # smaller transition system
#         [1.0, 2.5, 0.0],
        [1.227090539137495, 1.5261366178286393, 0],
        [0.6683476856850583, 0.6968077181658885, 0],
        [1.3618722811762958, 0.7142604793362379, 0],
        [2.059680461093005, 0.9722647716781803, 0],
        [2.935911437513193, 0.49039417072630315, 0],
        [3.5244797253701616, 1.0671446704710397, 0],
        [3.4171462977012945, 1.8636962642083317, 0],
        [3.2481431339499154, 2.580396624830811, 0],
        [2.6199028579323596, 2.5050497919480206, 0],
        [2.143319573133728, 3.0814698610803655, 0],
        [1.3410947476176465, 3.1958386839694723, 0],
        [1.0, 2.5, 0.0]
    ]*10)
    
#     origin = np.array([-0.08, 0.696, 0])
    origin = np.array([0, 0, 0])
    
    path = [p - origin for p in path]
    
    print 'Init:', path[0]
#     return
#     figure = plt.figure()
#     figure.add_subplot('111')
#     axes = figure.axes[0]
#     axes.axis('equal') # sets aspect ration to 1
#      
#     b = np.array(env['boundary']['limits'])
#     plt.xlim(b.T[0] - origin[0])
#     plt.ylim(b.T[1] - origin[1])
#        
#     mission.draw_environment(axes, figure, origin=origin[:2])
#      
#     px, py, _ = np.array(path).T
#      
#     plt.plot(px, py, 'ko-')
#      
#     plt.show()
#     return
#     
#     path = [(x, y), (x+1, y+1), (x+2, y+1), (x+1, y), (x, y)]
    experiment = 70
    with open('experiment_data_{}.txt'.format(experiment), 'a') as fout:
        t = time.time()
        print>>fout, '#', experiment, '#', t, '#', str(datetime.datetime.fromtimestamp(t))
    
    for xg, yg, _ in path:
        start = SE2BeliefState(xs, ys, theta, cov=covs)
        goal = SE2BeliefState(xg, yg, np.pi/2) # NOTE: end orientation does not matter
        
        print 'Start:', start.conf, 'Goal:', goal.conf
        
        tlqr = SwitchingController(robot, cam, goal,
                                   stateWeight=np.diag([1, 1, 200]),
                                   controlWeight=np.eye(2))
        ekf = ExtendedKF(cam, robot)
        controller = Controller(robot, cam, tlqr, ekf, withNoise=True) #FIXME: withNoise option is not ok
        
        process = save_trajectory()
        process.next()
        _, (trajectory, true_trajectory, opti_trajectory, quad_traj) = controller.execute(start, process)
        robot.execute([0, 0])
        print len(trajectory)
         
        print
        print 'Trajectories:'
        with open('experiment_data_{}.txt'.format(experiment), 'a') as fout:
            for p, tp, op, qp in zip(trajectory, true_trajectory, opti_trajectory, quad_traj):
                print 'C:', p.conf, 'T:', tp, 'O:', op
                print>> fout, '{', '"x": {}, "cov": {}, "filter": {}, "opti" : {}, "quad": {}'.format(list(p.conf), list(np.diag(p.cov)), list(tp), list(op), list(qp)), '}'
            print
            print>>fout
            print 'Start:', xs, ys, theta
            updated_state = trajectory[-1]
            print 'Before filtering', updated_state.conf, 'True:', cam.getObservation(updated_state)
            for _ in range(filter_stab):
                updated_state = controller.evolve(updated_state, 0, nocontrol=True)
                print>> fout, '{', '"x": {}, "cov": {}'.format(list(updated_state.conf), list(np.diag(updated_state.cov))), '}'
            xs, ys, theta = updated_state.conf
            covs = updated_state.cov
            print 'End:', xs, ys, theta, 'True:', cam.getObservation(updated_state)
            print
            print
            print>>fout
            print>>fout

#     start = SE2BeliefState(x, y, theta)
#     goal = SE2BeliefState(x+1, y+1, np.pi/2) # NOTE: end orientation does not matter
#      
#     tlqr = SwitchingController(robot, cam, goal,
#                                stateWeight=np.diag([1, 1, 200]),
#                                controlWeight=np.eye(2))
#     ekf = ExtendedKF(cam, robot)
#     
#     passed = 0
#     total = 0
#      
#     state = start
#      
# #     target = [x, y]
#     while not rospy.is_shutdown(): # condition
#         if True: #target is not None:
#             # 1. read sensors -- OptiTrack
# #             opti = OptiTrackInterface()
# #             body = opti.trackInfo()[rid] # get data for rigid body w/ rigid body ID `rid'
# #             opti._close()
# #             x, y, theta = body.z, body.x, np.deg2rad(body.yaw)
# #             print body.yaw, body.pitch, body.roll
#             x, y, theta = cam.getObservation(None)
# #             point = np.array([x, y])
#             print 'State (optitrack):', x, y, theta
#             print 'State (filter):', state.conf
#              
# #             control = tlqr.generateFeedbackControl(SE2BeliefState(x, y, theta))
#             control = tlqr.generateFeedbackControl(state)
#             linearVelocity, angularVelocity = control
#             print 'Actual Control:', (linearVelocity, angularVelocity)
#              
#             #-------------------------------------------------------------------
#              
#             # apply control
# #             nextState = robot.evolve(state, control, w=None)
#             print 'control=', control
#             # send commands to the robot
#             robot.execute([linearVelocity, angularVelocity])
#              
#             z = cam.getObservation(None)
# #             print 'nextState', nextState.conf
#             # apply filter and get state estimate
#             currentLS = tlqr.getCurrentLS(0)
#             nextLS = tlqr.getNextLS(1)
#             state = ekf.evolve(state, control, z, currentLS, nextLS)
#             print 'filter state:', state.conf
#             print
#              
# #             # 2. compute control values -- speeds for the two motors
# #             dist = norm(target - point)
# #             print "Distance: ", dist
# #             if dist < d_epsilon: # if target was reached, switch to next one
# #                 print "WITHIN EPSILON RANGE"
# #                 target = None
# #                 linearVelocity, angularVelocity = 0, 0
# #             else:
# #                 # compute heading of line of sight vector
# #                 vector = target - point
# #                 losAngle = np.arctan2(vector[1], vector[0])
# # #                 losAngle = np.rad2deg(losAngle)
# #                 
# #                 # compute error values for distance and heading
# #                 errorDist = dist
# #                 
# #                 errorHeading = losAngle - theta
# #                 if errorHeading > np.pi:
# #                     errorHeading = -(np.pi - errorHeading)
# #                 elif errorHeading < -np.pi:
# #                     errorHeading = np.pi + errorHeading
# #                 
# #                 print 'Target:', target
# #                 print 'Current position:', x, y, theta
# #                 print 'Distance error:', errorDist
# #                 print 'losAngle: ', losAngle
# #                 print 'theta: ', theta
# #                 print 'Heading error:', errorHeading
# #                 
# #                 # compute linear and angular velocities
# #                 if abs(errorHeading) < a_epsilon:
# #                     print 'Linear'
# #                     linearVelocity = 1 * errorDist # use PID for linear velocity
# #                     linearVelocity = max(min(linearVelocity, 0.4), 0.12)
# #                     angularVelocity = 5 * errorHeading
# # #                     angularVelocity = max(min(angularVelocity, 10), -10)
# #                 else:
# #                     print 'Rotate on spot'
# #                     linearVelocity = 0 # use constant linear velocity
# #                     angularVelocity = 5 * errorHeading
# #                 print 'Actual Control:', (linearVelocity, angularVelocity)
# #                 print 'Controller output:', tlqr.generateFeedbackControl(SE2BeliefState(x, y, theta))
# #                 currentLS = tlqr.getCurrentLS(0)
# #                 nextLS = tlqr.getNextLS(1)
# #                 print 'Controller output filter:', tlqr.generateFeedbackControl(
# #                             ekf.evolve(SE2BeliefState(x, y, theta),
# #                             np.array((linearVelocity, angularVelocity)),
# #                             np.array([x, y, theta]), currentLS, nextLS))
# #                 
# #                 test_res = np.allclose((linearVelocity, angularVelocity),
# #                                   tlqr.generateFeedbackControl(SE2BeliefState(x, y, theta)),
# #                                   rtol=0, atol=1e-4)
# #                 passed += test_res
# #                 total += 1
# #                 print 'Same:', test_res
# #                 print
# #                 
# #                 currentLS = tlqr.getCurrentLS(0)
# #                 nextLS = tlqr.getNextLS(1)
# #                 print 'filter:', ekf.evolve(SE2BeliefState(x, y, theta),
# #                                   np.array((linearVelocity, angularVelocity)),
# #                                   np.array([x, y, theta]), currentLS, nextLS)
# #                 
# #                 print
#         else:
#             linearVelocity, angularVelocity = 0, 0
#      
#     # stop robot
#     robot.execute([0, 0])
#      
#     print 'Passed:', passed, '/', total

def test_filter():
#     from optitrack import OptiTrackInterface
    from numpy.linalg import norm
    import time
    
    isSimulation = False
    
    if not isSimulation:
        rospy.init_node("ControllerTest", anonymous=True)
    
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    logging.info('\n' + str(mission))
    logging.info('Seed: %s', mission.simulation.get('seed', None))
     
    # load environment
    env = mission.environment
    # load robot model
    assert mission.no_active_vehicles == 1
    robotModel = mission.vehicles.itervalues().next()
    print 'Motion model:', robotModel['motion_model']
    
    robot = UnicycleMotionModel(robotModel, isSimulation)
    print 'Time step:', robot.dt
    robot.rate.sleep()
    
    print 'Observation model:', robotModel['observation_model']
    cam = CameraLocalization(robotModel, env, isSimulation=isSimulation)
    robot.rate.sleep()
    
    #---------------------------------------------------------------------------
    x, y, theta = cam.getObservation(None) #body.z, body.x, np.deg2rad(body.yaw)
    print x, y, theta
    #---------------------------------------------------------------------------
    
    def save_trajectory():
        true_traj = []
        traj = []
        while True:
            state = yield traj, true_traj
            print 'save state:', state.conf
            traj.append(state.copy())
            true_traj.append(cam.getObservation(state)) #TODO: change this back
#             true_traj.append(cam.getObservationPrediction(state))
    
    goal = SE2BeliefState(x, y, 0)
    tlqr = SwitchingController(robot, cam, goal,
                                   stateWeight=np.diag([1, 1, 200]),
                                   controlWeight=np.eye(2))
    
    ekf = ExtendedKF(cam, robot)
    updated_state = SE2BeliefState(x, y, theta, cov=np.eye(3))
    
    print 'Before filtering', updated_state.conf, 'True:', cam.getObservation(updated_state)
    control = np.array([0, 0])
    for _ in it.count():
        z = cam.getObservation(updated_state)
        # apply filter and get state estimate
        currentLS = tlqr.getCurrentLS(0)
        nextLS = tlqr.getNextLS(1)
        updated_state = ekf.evolve(updated_state, control, z, currentLS, nextLS)
        
        print 'Filter:', updated_state.conf, 'True:', z
        
        time.sleep(0.25)
        
        if rospy.is_shutdown():
            break
    
    

def test_send():
    from geometry_msgs.msg import Vector3
    
    rospy.init_node('MotionModel', anonymous=True)
    pub = rospy.Publisher('GroundCommandX80Pro1', Vector3, queue_size=10)
    
    rate = rospy.Rate(1) # Hz
    rate.sleep()
    
    print 'msg 1'
    msg = Vector3(0.1, 0.1, 0)
    pub.publish(msg)
    rate.sleep()
    
    print 'msg 2'
    msg = Vector3(0, 0, 0)
    pub.publish(msg)
    rate.sleep()
    
    print 'Done!'

if __name__ == '__main__':
    logfile = 'controller_test.log'
    verbose=True
    # configure logging
    fs, dfs = '%(asctime)s %(levelname)s %(message)s', '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG
    logging.basicConfig(filename=logfile, level=loglevel, format=fs, datefmt=dfs)
    if verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)
    
    pass
#     test_unicycle_model()
#     test_omnidirectional_model()
    test_controller()
#     test_controller__()
#     test_controller_offaxis()
#     test_off_axis_point()
#     test_send()
#     test_filter()
