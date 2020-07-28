#!/usr/bin/env python

"""Script to test path planning algorithm
"""
# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import

import os
import time
import yaml

# pylint: disable=invalid-name
# pylint: disable=import-error
# pylint: disable=wrong-import-position
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
traj_path = os.path.join(parentdir, 'scripts')
os.sys.path.insert(0, traj_path)

from trajectory_solver import TrajectorySolver
# pylint: enable=invalid-name
# pylint: enable=import-error
# pylint: enable=wrong-import-position

from demos_positions import *
from demos_formations import formation_demo
from demos_random import random_pos

def demo():
    """Launch trajectories tests
    """

    agents = []
    obstacles = []
    arena_max = 5

    # Choose demo to execute
    agents = demo_two_agents()
    # agents = demo_two_agents_vert()
    # agents, obstacles = demo_wall()
    # agents, obstacles = through_wall()

    # agents = corners_2()
    # agents = corners_2_2()
    # agents = corners_4()
    # agents = corners_6()

    # agents = seven_agents()

    # agents, arena_max = random_pos(9, 1, seed=None)
    # agents = formation_demo(9, "v")

    start_time = time.time()
    solver = TrajectorySolver(agents, SOLVER_ARGS, verbose=True)
    solver.set_obstacles(obstacles)

    solver.wait_for_input(False)
    solver.set_slow_rate(1.0)
    solver.set_arena_max(arena_max)

    solver.solve_trajectories()

    print "Compute time:", (time.time() - start_time)*1000, "ms"

    solver.plot_trajectories()

if __name__ == '__main__':
    # Read arguments from yaml file
    PARENT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    FILE_PATH = os.path.join(PARENT_DIR, 'conf.yaml')

    with open(FILE_PATH) as f:
        YAML_CONF = yaml.load(f, Loader=yaml.FullLoader)

    SOLVER_ARGS = YAML_CONF['trajectory_solver']

    AGENT_ARGS = {'r_min': SOLVER_ARGS['r_min'],
                  'col_radius_ratio': SOLVER_ARGS['col_radius_ratio'],
                  'goal_thres': SOLVER_ARGS['goal_thres']}

    demo()
