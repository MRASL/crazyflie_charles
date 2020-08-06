#!/usr/bin/env python

"""Script to find performance of current conf
"""

# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import

import os
import time
import numpy as np
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

def benchmark_algo():
    """To benchmark current performances of algorithm

    Stats:
        - Success rate
        - Total compute time
        - Total travel time

    Tests:
        - corners_4
        - demo_wall (wall from (2.0, -1.0, 0.) to (2.0, 2.5, 0.0))
        - seven_agents
        - 9 agents (seed: 6441753598703859782L, density: 1)
        - 15 agents (seed: 7125329410299779625L, density: 1)
        - 25 agents (seed: 8430841635042043371L, density: 1)
        - 50 agents (seed: 3963364070630474782L, density: 1)
        - square formation, 9 agents
        - v formation, 9 agents
    """
    test_to_run = [(corners_4, (), {}, "corners_4"),
                   (demo_wall, (), {'wall_coords':[(2.0, -1.0, 0.), (2.0, 2.5, 0.0)]}, "demo_wall"),
                   (seven_agents, (), {}, "seven_agents"),
                   (random_pos, (9, 1), {'seed':6441753598703859782L}, "random_pos_9"),
                   (random_pos, (15, 1), {'seed':7125329410299779625L}, "random_pos_15"),
                   (random_pos, (25, 1), {'seed':8430841635042043371L}, "random_pos_25"),
                   (random_pos, (50, 1), {'seed':3963364070630474782L}, "random_pos_50"),
                   (formation_demo, (9, "square"), {}, "formation_square"),
                   (formation_demo, (9, "v"), {}, "formation_v"),]
    tests_data = {}

    start_time = time.time()

    for each_test in test_to_run:
        test_name = each_test[3]
        print "Running test: %s" % test_name
        success, travel_time, compute_time = run_test(each_test)

        print "\tSucces: %s" % success
        print "\tCompute time: %.2f sec" % compute_time
        print "\tTravel Time: %.2f sec" % travel_time
        print "\n"

        tests_data[test_name] = {"success": success,
                                 "compute_time": compute_time,
                                 "travel_time": travel_time}


    success_rate, compute_sum, travel_sum = compute_global_perfo(tests_data)

    print "Results:"
    print "\tSuccess rate: %.2f%%" % success_rate
    print "\tCompute Time Total: %.2f sec" % compute_sum
    print "\tTravel Time Total: %.2f sec" % travel_sum
    print "\tTotal Time: %.2f sec" % (time.time() - start_time)

def run_test(test_info):
    """Run a test and compute test performance

    Args:
        test_info (list): test_func, args, kwargs, test_name

    Returns:
        :obj:`bool`: True if trajectory found
        :obj:`float`: Travel time (sec)
        :obj:`float`: Compute time (sec)
    """
    agents = []
    obstacles = []
    arena_max = 5

    func = test_info[0]
    args = test_info[1]
    kwargs = test_info[2]
    res = func(*args, **kwargs)

    if isinstance(res[0], list):
        agents = res[0]

        if isinstance(res[1], list):
            obstacles = res[1]
        else:
            arena_max = res[1]
    else:
        agents = res

    test_start_time = time.time()
    solver = TrajectorySolver(agents, SOLVER_ARGS, verbose=False)
    solver.set_obstacles(obstacles)

    solver.trajectory_plotter.set_wait_for_input(False)
    solver.trajectory_plotter.set_dot_plotting(False)
    solver.trajectory_plotter.set_slow_rate(1.0)
    solver.trajectory_plotter.set_axes_limits(arena_max, arena_max)

    success, travel_time = solver.solve_trajectories()

    compute_time = (time.time() - test_start_time)

    return success, travel_time, compute_time

def compute_global_perfo(tests_data):
    """Compute global performances.

    Finds success rate, total compute time and total travel time

    Args:
        tests_data (dict): Perfo of each test

    Returns:
        :obj:`float`: Success rate (%)
        :obj:`float`: Total compute time (sec)
        :obj:`float`: Total travel time (sec)
    """
    success_list = []
    compute_time_list = []
    travel_time_list = []

    for _, each_test_res in tests_data.items():
        success_list.append(each_test_res["success"])
        compute_time_list.append(each_test_res["compute_time"])
        travel_time_list.append(each_test_res["travel_time"])

    success_rate = success_list.count(True)/float(len(success_list))*100
    compute_sum = np.sum(compute_time_list)
    travel_sum = np.sum(travel_time_list)

    return success_rate, compute_sum, travel_sum

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


    benchmark_algo()
