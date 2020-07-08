#!/usr/bin/env python

"""Script to test path planning algorithm
"""
# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import

import time
import random as rand
import numpy as np
from numpy import array, mean
from numpy.linalg import norm

from trajectory_solver import TrajectorySolver
from agent import Agent

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
    # agents, obstacles = demo_wall()
    # agents, obstacles = through_wall()

    # agents = corners_2()
    # agents = corners_2_2()
    # agents = corners_4()
    # agents = corners_6()

    # agents = seven_agents()
    # agents = nine_agents()

    # agents, arena_max = random_pos(9, 1)
    agents = formation_demo(9, "square")

    start_time = time.time()
    solver = TrajectorySolver(agents)
    solver.set_obstacles(obstacles)

    solver.wait_for_input(False)
    solver.set_slow_rate(1.0)
    solver.set_arena_max(arena_max)

    solver.solve_trajectories()

    print "Compute time:", (time.time() - start_time)*1000, "ms"

    solver.plot_trajectories()

# Random positions
def random_pos(n_agents, density):
    """Compute starting and final positions of agents

    Args:
        n_agents (int): Number of agents
        density (float): Agents/m**3
    """
    min_distance = 0.7 #: float: min distance between two agents starting pos or goal
    total_vol = n_agents/density
    max_coord = np.sqrt(total_vol)

    agent_list = []  #: list of Agent
    start_list = [] #: list of [x, y, z]
    goal_list = [] #: list of [x, y, z]

    for _ in range(n_agents):
        new_agent = Agent()

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
    """Check algorithm performances"""

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

if __name__ == '__main__':
    demo()
    # update_test()
    # algo_performance(4, 1, 30)  #: n_agents, density, n_tests

    # Results (4, 1, 10):
    # 80%, 3075
    # 50%, 2201
    # 70%, 2211

    # Updated algo (4, 1, 20):
    # 100%, 737

    # Results (9, 1, 10):
    # 0%, 8444
