#!/usr/bin/env python

"""Script to test path planning algorithm
"""
# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import

import os
import sys
import time
import random as rand
import numpy as np
from numpy import array, mean
from numpy.linalg import norm
import yaml

# pylint: disable=invalid-name
# pylint: disable=import-error
# pylint: disable=wrong-import-position
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
traj_path = os.path.join(parentdir, 'scripts')
os.sys.path.insert(0, traj_path)

from trajectory_solver import TrajectorySolver
from agent import Agent
# pylint: enable=invalid-name
# pylint: enable=import-error
# pylint: enable=wrong-import-position

from demos_positions import *
from demos_formations import formation_demo

def demo():
    """Launch trajectories tests
    """

    agents = []
    obstacles = []
    arena_max = 5

    # Choose demo to execute
    # agents = demo_two_agents()
    agents, obstacles = demo_wall()
    # agents, obstacles = through_wall()

    # agents = corners_2()
    # agents = corners_2_2()
    # agents = corners_4()
    # agents = corners_6()

    # agents = seven_agents()

    agents, arena_max = random_pos(9, 1, seed=None)
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

# Random positions
def random_pos(n_agents, density, seed=None):
    """Compute starting and final positions of agents

    Args:
        n_agents (int): Number of agents
        density (float): Agents/m**3
        seed (float): To specify seed
    """
    min_distance = 0.7 #: float: min distance between two agents starting pos or goal
    total_vol = n_agents/density
    max_coord = np.sqrt(total_vol)

    agent_list = []  #: list of Agent
    start_list = [] #: list of [x, y, z]
    goal_list = [] #: list of [x, y, z]

    if seed is None:
        seed = rand.randrange(sys.maxsize)

    rand.seed(seed)
    print "Seed used:", seed

    for _ in range(n_agents):
        new_agent = Agent(AGENT_ARGS)

        start = find_position_at_dist(max_coord, min_distance, start_list)
        goal = find_position_at_dist(max_coord, min_distance, goal_list)

        new_agent.set_starting_position(start)
        new_agent.set_goal(goal)

        agent_list.append(new_agent)
        start_list.append(start)
        goal_list.append(goal)

    return agent_list, max_coord*1.1

def find_position_at_dist(max_coord, min_dist, other_positions):
    """Find a random position.

    Make sure the position is farther than min_dist from other positions and in the cube
    (0, 0, 0) -> (max_coord, max_coord, max_coord)

    Fails if no solution is found after 50 iterations

    Args:
        max_coord (float): Max coordinate possible [m]
        min_dist (float): Min distance [m]
        other_positions (list of 3x1 array): List of other positions

    Returns:
        3x1 array: Random position
    """
    pos_found = False
    count = 0
    while not pos_found:
        if count == 50:
            raise RuntimeError('No valid position found, try decreasing density')

        position = array([rand.random(), rand.random(), 0.0])
        position = position*max_coord
        pos_found = True

        for each_other_pos in other_positions:
            dist = norm(position - each_other_pos)

            if dist < min_dist:
                pos_found = False

        count += 1

    return position

# Performances
def algo_performance(n_agents, density, n_tests):
    """Check algorithm performances for random configurations"""

    time_list = []
    res_list = []

    for i in range(1, n_tests + 1):
        print "Test %i:" % i

        agents = []

        agents, arena_max = random_pos(n_agents, density)

        solver = TrajectorySolver(agents, verbose=False)
        solver.set_arena_max(arena_max)

        start_time = time.time()
        res = solver.solve_trajectories()
        compute_time = (time.time() - start_time)*1000

        time_list.append(compute_time)
        res_list.append(res)

        print "\tResult: %s" % res
        print "\tCompute time: %.2f ms" % compute_time

    time_average = mean(time_list)
    success_average = res_list.count(True)/float(len(res_list))*100

    print '\n'
    print 'Success rate: %.2f%%' % success_average
    print 'Compute time average: %.2f ms' % time_average

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
    test_to_run = [(corners_4, (), {}),
                   (demo_wall, (), {}),
                   (seven_agents, (), {}),
                   (random_pos, (9, 1), {'seed':6441753598703859782L}),
                   (random_pos, (15, 1), {'seed':7125329410299779625L}),
                   (random_pos, (25, 1), {'seed':8430841635042043371L}),
                   (random_pos, (50, 1), {'seed':3963364070630474782L}),
                   (formation_demo, (9, "square"), {}),
                   (formation_demo, (9, "v"), {}),]

    for each_test in test_to_run:
        print "Running test: %s" % str(each_test[0].__name__)
        agents = []
        obstacles = []
        arena_max = 5

        func = each_test[0]
        args = each_test[1]
        kwargs = each_test[2]
        res = func(*args, **kwargs)

        if isinstance(res[0], list):
            agents = res[0]

            if isinstance(res[1], list):
                obstacles = res[1]
            else:
                arena_max = res[1]
        else:
            agents = res

        start_time = time.time()
        solver = TrajectorySolver(agents, SOLVER_ARGS, verbose=True)
        solver.set_obstacles(obstacles)

        solver.wait_for_input(False)
        solver.set_slow_rate(1.0)
        solver.set_arena_max(arena_max)

        solver.solve_trajectories()

        print "Compute time:", (time.time() - start_time)*1000, "ms"
        print "\n"
        # solver.plot_trajectories()


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

    # demo()

    benchmark_algo()

    # update_test()
    # algo_performance(4, 1, 30)  #: n_agents, density, n_tests
