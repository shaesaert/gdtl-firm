'''
Created on Mar 4, 2016

@author: cristi
'''

import numpy as np
import matplotlib.pyplot as plt

from firm import Mission


def draw_experiment_data(experiment, viewport=None, estimate=True, camera=True, truth=True, cov=False):
    with open('/home/cristi/Dropbox/FIRM/Experiments/iser_trials/experiment_data_{}.txt'.format(experiment), 'r') as fin:
        for line in fin:
            line = line.strip()
            if line.startswith('#'):
                pass
            elif line:
                data = dict(eval(line))
                if data.has_key('opti') and data.has_key('filter'):
                    plot_data = []
                    if estimate:
                        plot_data.append((data['x'], (1.0, 0, 0)))
                    if camera:
                        plot_data.append((data['filter'], (1, 1, 0)))
                    if truth:
                        plot_data.append((data['opti'], (0, 1, 0))) # TODO: colors 'g'
                    
                    for (x, y, yaw,), col in plot_data:
                        plt.arrow(x, y, 0.1*np.cos(yaw), 0.1*np.sin(yaw),
                                  hold=True, color=col)
                    
                    if cov:
                        viewport.add_patch( Mission.covariance_ellipse(
                            data['x'][:2], np.diag(data['cov'][:2]), color='b',
                            fill=False, lw=1, zorder=0))

def plot_experiments():
    
    # 1. load mission and environment
#     mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/mission.yaml')
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
    
    for experiment in range(1006, 1012):
        draw_experiment_data(experiment, estimate=False, camera=False, truth=True)
    
#     experiment = 1010
#     draw_experiment_data(experiment)
    
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.tight_layout(pad=0.5)
    
    for img in ['png', 'jpg', 'pdf', 'svg']:
#         plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/experiment_{}.{}'.format(experiment, img))
        plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/all_experiments_ground_truth_only.{}'.format(img))
    plt.show()

def plot_experiments_cov():
    
    # 1. load mission and environment
#     mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/simple/mission.yaml')
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/mission.yaml')
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
    
#     for experiment in range(1002, 1012):
#         draw_experiment_data(experiment, axes, estimate=False, camera=False, truth=False, cov=True)
    
    experiment = 1010
    draw_experiment_data(experiment, axes, estimate=True, camera=True, truth=True, cov=True)
    
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.tight_layout(pad=0.5)
    
    for img in ['png', 'jpg', 'pdf', 'svg']:
#         plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/experiment_estimate_and_cov_{}.{}'.format(experiment, img))
        plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/experiment_all_pose_and_cov_{}.{}'.format(experiment, img))
#         plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/experiment_cov_{}.{}'.format(experiment, img))
#         plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/all_experiments_ground_truth_only.{}'.format(img))
    plt.show()

def plot_cov():
    plt.figure()
    dt = 0.5
    
    plot_data = []
    for experiment in range(1006, 1012):
        with open('/home/cristi/Dropbox/FIRM/Experiments/iser_trials/experiment_data_{}.txt'.format(experiment), 'r') as fin:
            for line in fin:
                line = line.strip()
                if line.startswith('#'):
                    pass
                elif line:
                    data = dict(eval(line))
                    if data.has_key('opti') and data.has_key('filter'):
                        if np.sum(data['cov']) < 2.99:
                            plot_data.append(np.sum(data['cov']))
    
    print len(plot_data)
    
    plt.plot(np.arange(0, len(plot_data))*dt, plot_data)
    
    plt.xlabel('t [sec]')
    plt.ylabel('tr(P) [m$^2$]')
    plt.tight_layout(pad=0.5)
    
    for img in ['png', 'jpg', 'pdf', 'svg']:
        plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/experiment_cov_vs_time_{}.{}'.format(experiment, img))
    plt.show()

if __name__ == '__main__':
#     plot_experiments()
#     plot_experiments_cov()
    plot_cov()
