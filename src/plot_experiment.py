'''
Created on Mar 4, 2016

@author: cristi
'''

import numpy as np
import matplotlib.pyplot as plt

from firm import Mission

def plot_experiments():
    
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
#     logging.info('\n' + str(mission))
#     logging.info('Seed: %s', mission.simulation.get('seed', None))
     
    # load environment
    env = mission.environment
    
#     origin = np.array([-0.08, 0.696, 0])
    origin = np.array([0, 0, 0])
    
    
    figure = plt.figure()
    figure.add_subplot('111')
    axes = figure.axes[0]
    axes.axis('equal') # sets aspect ration to 1
    
    b = np.array(env['boundary']['limits'])
    plt.xlim(b.T[0] - origin[0])
    plt.ylim(b.T[1] - origin[1])
    
    mission.draw_environment(axes, figure, origin=origin[:2])
    
    experiment = 60
    with open('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/experiments/experiment_data_{}.txt'.format(experiment), 'r') as fin:
        for line in fin:
            line = line.strip()
            if line.startswith('#'):
                pass
            elif line:
                data = dict(eval(line))
                if data.has_key('opti') and data.has_key('filter'):
                    robot_pose = data['x']
                    cam_pose = data['filter']
                    opti_pose = data['opti']
                    
                    for (x, y, yaw,), col in zip([robot_pose, cam_pose, opti_pose], ['k', 'y', 'g']):
                        plt.arrow(x, y, 0.1*np.cos(yaw), 0.1*np.sin(yaw), hold=True, color=col)
    
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.show()

if __name__ == '__main__':
    plot_experiments()