'''
Created on Jul 13, 2016

@author: cristi
'''
import itertools as it


import numpy as np
from scipy.spatial.distance import euclidean as dist
import matplotlib.pyplot as plt
import yaml

from lomap import Ts
from firm import Mission


def draw_ts(ts, viewport, figure):
    if not ts.g.number_of_nodes():
        return
    # draw nodes
    x, y = zip(*[s[:2] for s in ts.g.nodes_iter()])
    viewport.plot(x, y, 'o', color='w') # TODO: node color
    # draw transitions
    for start, stop, d in ts.g.edges_iter(data=True):
        if start != stop:
            color = d.get('color', 'w') #TODO: edge color
            path = [start, stop] # TODO: extract nominal path
            line_style = d.get('line_style', '-') # TODO: line style
            for u, v in it.izip(path[:-1], path[1:]):
                x, y = np.array([u[:2], v[:2]]).T
                viewport.plot(x, y, color=color, linestyle=line_style)

def extract_ts():
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/mission.yaml')
#     logging.info('\n' + str(mission))
#     logging.info('Seed: %s', mission.simulation.get('seed', None))
     
    # load environment
    env = mission.environment
    
    # load transition system
    ts_filename = '/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/transition_system.txt'
    with open(ts_filename, 'r') as fin:
        data = yaml.load(fin)
    
    
    ts = Ts(multi=False)
    ts.g.add_edges_from(data['ts'])
    ts.init[data['ts_init']] = 1
    
    figure = plt.figure()
    figure.add_subplot('111')
    axes = figure.axes[0]
    axes.axis('equal') # sets aspect ration to 1
    
    b = np.array(env['boundary']['limits'])
    plt.xlim(b.T[0])
    plt.ylim(b.T[1])
    
    mission.draw_environment(axes, figure, origin=np.array((0, 0)))
    draw_ts(ts, axes, figure)
    
    policy_filename = '/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/policy_new.txt'
    with open(policy_filename, 'w') as fout:
        print>>fout, '    path = np.array(['
    
    def onclick(event):
        print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              (event.button, event.x, event.y, event.xdata, event.ydata))
        
        node, d = min([(p, dist(p[:2], (event.xdata, event.ydata)))
                        for p in ts.g.nodes_iter()],
                   key=lambda x: x[1])
        print node, d
        assert ts.g.has_node(node)
        
        with open(policy_filename, 'a') as fout:
            print>>fout, '        {},'.format(node)
    
    cid = figure.canvas.mpl_connect('button_press_event', onclick)
    
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.tight_layout(pad=0.5)
    plt.show()
    
    with open(policy_filename, 'a') as fout:
        print>>fout, '        ]*10)'

def plot_ts():
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/mission.yaml')
#     logging.info('\n' + str(mission))
#     logging.info('Seed: %s', mission.simulation.get('seed', None))
     
    # load environment
    env = mission.environment
    
    # load transition system
    ts_filename = '/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/transition_system.txt'
    with open(ts_filename, 'r') as fin:
        data = yaml.load(fin)
    
    
    ts = Ts(multi=False)
    ts.g.add_edges_from(data['ts'])
    ts.init[data['ts_init']] = 1
    
    figure = plt.figure()
    figure.add_subplot('111')
    axes = figure.axes[0]
    axes.axis('equal') # sets aspect ration to 1
    
    b = np.array(env['boundary']['limits'])
    plt.xlim(b.T[0])
    plt.ylim(b.T[1])
    
    mission.draw_environment(axes, figure, origin=np.array((0, 0)))
    draw_ts(ts, axes, figure)
    
    path = None
    with open('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/policy_new.txt', 'r') as fin:
        lines = fin.readlines()
    lines = ''.join(map(str.strip, lines))
    exec lines
    
    print path
    
    axes.plot([p[0] for p in path], [p[1] for p in path], 'o', color=(1, 0.5, 0), markersize=10)
    for u, v in it.izip(path[:-1], path[1:]):
        x, y = np.array([u[:2], v[:2]]).T
        axes.plot(x, y, color=(1, 0.5, 0), linestyle='-', lw=4)
    
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.tight_layout(pad=0.5)
    
    for img in ['png', 'jpg', 'pdf', 'svg']:
        plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/bu_ts.{}'.format(img))
    
    plt.show()

def plot_policy():
    # 1. load mission and environment
    mission = Mission.from_file('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/mission.yaml')
#     logging.info('\n' + str(mission))
#     logging.info('Seed: %s', mission.simulation.get('seed', None))
     
    # load environment
    env = mission.environment
    
    figure = plt.figure()
    figure.add_subplot('111')
    axes = figure.axes[0]
    axes.axis('equal') # sets aspect ration to 1
    
    b = np.array(env['boundary']['limits'])
    plt.xlim(b.T[0])
    plt.ylim(b.T[1])
    
    mission.draw_environment(axes, figure, origin=np.array((0, 0)))
    
    path = None
    with open('/home/cristi/Dropbox/work/workspace_linux/PyFIRM/src/data/bu/policy_new.txt', 'r') as fin:
        lines = fin.readlines()
    lines = ''.join(map(str.strip, lines))
    exec lines
    
    print path
    
    axes.plot([p[0] for p in path], [p[1] for p in path], 'o', color=(1, 0.5, 0), markersize=10)
    for u, v in it.izip(path[:-1], path[1:]):
        x, y = np.array([u[:2], v[:2]]).T
        axes.plot(x, y, color=(1, 0.5, 0), linestyle='-', lw=4)
    
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.tight_layout(pad=0.5)
    
#     for img in ['png', 'jpg', 'pdf', 'svg']:
#         plt.savefig('/home/cristi/Dropbox/FIRM/Experiments/bu_case/bu_policy.{}'.format(img))
    
    plt.show()

if __name__ == '__main__':
#     extract_ts()
    plot_ts()
#     plot_policy()
    
