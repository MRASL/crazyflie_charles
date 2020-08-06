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


    """
    Uncomment demo to execute
    """

    # agents = demo_two_agents()
    # agents = demo_two_agents_vert()
    # agents, obstacles = demo_wall([(2.0, -1.0, 0.), (2.0, 2.5, 0.0)])
    # agents, obstacles = through_wall()

    # agents = corners_2()
    # agents = corners_2_2()
    # agents = corners_4()
    # agents = corners_6()

    # agents = seven_agents()

    # agents, arena_max = random_pos(9, 1, seed=None)

    #! Perfo tests
    # agents = corners_4()
    # agents, obstacles = demo_wall([(2.0, -1.0, 0.), (2.0, 2.5, 0.0)])
    # agents = seven_agents()
    # agents, arena_max = random_pos(9, 1, seed=6441753598703859782L)
    # agents, arena_max = random_pos(15, 1, seed=7125329410299779625L)
    # agents, arena_max = random_pos(25, 1, seed=8430841635042043371L)
    # agents, arena_max = random_pos(50, 1, seed=3963364070630474782L)
    # agents = formation_demo(9, "square")
    # agents = formation_demo(9, "v")

    start_time = time.time()
    solver = TrajectorySolver(agents, SOLVER_ARGS, verbose=True)
    solver.set_obstacles(obstacles)

    solver.trajectory_plotter.set_wait_for_input(False)
    solver.trajectory_plotter.set_dot_plotting(True)
    solver.trajectory_plotter.set_slow_rate(1.0)
    solver.trajectory_plotter.set_axes_limits(arena_max, arena_max)

    solver.solve_trajectories()

    print "Compute time:", (time.time() - start_time)*1000, "ms"

    solver.plot_trajectories()

if __name__ == '__main__':
    # Read arguments from yaml file
    PARENT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    FILE_PATH = os.path.join(PARENT_DIR, 'swarm_manager/conf/swarm_conf.yaml')

    with open(FILE_PATH) as f:
        YAML_CONF = yaml.load(f, Loader=yaml.FullLoader)

    SOLVER_ARGS = YAML_CONF['trajectory_solver']

    AGENT_ARGS = {'r_min': SOLVER_ARGS['r_min'],
                  'col_radius_ratio': SOLVER_ARGS['col_radius_ratio'],
                  'goal_dist_thres': SOLVER_ARGS['goal_dist_thres'],
                  'goal_speed_thres': SOLVER_ARGS['goal_speed_thres'],
                 }

    demo()
