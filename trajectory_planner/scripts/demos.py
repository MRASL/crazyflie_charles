#!/usr/bin/env python

"""Script to test path planning algorithm
"""

import time
import random as rand
import numpy as np
from numpy import array, mean
from numpy.linalg import norm

from trajectory_solver import TrajectorySolver
from agent import Agent

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

    agents, arena_max = random_pos(9, 1)

    start_time = time.time()
    solver = TrajectorySolver(agents)
    solver.set_obstacles(obstacles)

    solver.wait_for_input(False)
    solver.set_slow_rate(1.0)
    solver.set_arena_max(arena_max)

    solver.solve_trajectories()

    print "Compute time:", (time.time() - start_time)*1000, "ms"

    solver.plot_trajectories()

def demo_two_agents():
    """Two agents trading spots
    """
    a_1 = Agent(start_pos=[0.5, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    a_2 = Agent(start_pos=[4.0, 2.0, 0.0], goal=[0.5, 2.0, 0.0])

    return [a_1, a_2]

def demo_wall():
    """Wall"""
    a_1 = Agent(start_pos=[0.0, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    obs_coords = compute_obstacle([[(2.0, -1.0, 0.), (2.0, 3.5, 0.0)]], 15)

    return [a_1], obs_coords

def through_wall():
    """Wall"""
    a_1 = Agent(start_pos=[0.0, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    a_2 = Agent(start_pos=[5.0, 2.0, 0.0], goal=[0.0, 2.0, 0.0])

    obs_coords = compute_obstacle([[(2.0, -1.0, 0.), (2.0, 1.5, 0.0)],
                                   [(2.0, 2.5, 0.), (2.0, 5.0, 0.0)]], 30)

    return [a_1, a_2], obs_coords

def corners_2():
    """Two agents trading spots, starting from opposite corners
    """
    # a_1 = Agent(start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    # a_2 = Agent(start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])

    a_1 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_2 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    return [a_1, a_2]

def corners_2_2():
    """Corners 2, v2
    """
    a_1 = Agent(start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])

    return [a_1, a_2]

def corners_4():
    """Four agents, starting from opposite corners
    """
    a_1 = Agent(start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    return [a_1, a_2, a_3, a_4]

def six_agents():
    """Six agents
    """
    a_1 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[1.5, 3.0, 0.0])
    a_2 = Agent(start_pos=[2.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[1.0, 2.5, 0.0], goal=[4.0, 0.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])
    a_5 = Agent(start_pos=[2.5, 2.5, 0.0], goal=[4.0, 2.5, 0.0])
    a_6 = Agent(start_pos=[3.2, 3.2, 0.0], goal=[0.5, 0.0, 0.0])

    return [a_1, a_2, a_3, a_4, a_5, a_6]

def corners_6():
    """Six agents, starting from opposite corners
    """
    a_1 = Agent(start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    a_5 = Agent(start_pos=[2.0, 0.0, 0.0], goal=[2.0, 4.0, 0.0])
    # a_6 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])


    return [a_1, a_2, a_3, a_4, a_5]

def seven_agents():
    """Seven agents
    """
    a_1 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[1.5, 3.0, 0.0])
    a_2 = Agent(start_pos=[2.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[1.0, 2.5, 0.0], goal=[4.0, 0.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])
    a_5 = Agent(start_pos=[2.5, 2.5, 0.0], goal=[4.0, 2.5, 0.0])
    a_6 = Agent(start_pos=[3.2, 3.2, 0.0], goal=[0.5, 0.0, 0.0])

    a_7 = Agent(start_pos=[1.7, 0.5, 0.0], goal=[0.8, 3.8, 0.0])

    return [a_1, a_2, a_3, a_4, a_5, a_6, a_7]

def nine_agents():
    """Nine agents
    """
    a_1 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[1.5, 3.0, 0.0])
    a_2 = Agent(start_pos=[2.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[1.0, 2.5, 0.0], goal=[4.0, 0.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])
    a_5 = Agent(start_pos=[2.5, 2.5, 0.0], goal=[4.0, 2.5, 0.0])
    a_6 = Agent(start_pos=[3.2, 3.2, 0.0], goal=[0.5, 0.0, 0.0])

    a_7 = Agent(start_pos=[1.7, 0.5, 0.0], goal=[0.8, 3.8, 0.0])
    a_8 = Agent(start_pos=[0.1, 1.8, 0.0], goal=[4.0, 1.0, 0.0])
    # a_9 = Agent(start_pos=[1.7, 0.5, 0.0], goal=[0.8, 3.8, 0.0])

    return [a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8]

def update_test():
    """To test when agents position are changed after solver initialization
    """
    a_list = ['a1', 'a2', 'a3']
    agents = {}
    for each_a in a_list:
        agents[each_a] = Agent()

    agent_list = [agent for (_, agent) in agents.items()]

    solver = TrajectorySolver(agent_list)

    agents['a1'].set_starting_position([1.0, 1.0, 0.0])
    agents['a1'].set_goal([4.0, 4.0, 0.0])

    agents['a2'].set_starting_position([4.0, 4.0, 0.0])
    agents['a2'].set_goal([1.0, 1.0, 0.0])

    agents['a3'].set_starting_position([2.0, 0.5, 0.0])
    agents['a3'].set_goal([2.0, 4.0, 0.0])

    solver.update_agents_info()

    solver.solve_trajectories()
    solver.plot_trajectories()

def compute_obstacle(positions, n_pts):
    """Compute coordinates of a wall

    Args:
        position (list of list of list): [[obstacle1_start, obstacle1_end], [obstacle2]]; [x, y, z]
        n_pts (int): Number of pts

    Returns:
        list: All coords of wall
    """
    all_coords = []
    for each_obstacle in positions:
        obstacle_start = each_obstacle[0]
        obstacle_end = each_obstacle[1]

        coords = []
        obstacle_x = np.linspace(obstacle_start[0], obstacle_end[0], num=n_pts)
        obstacle_y = np.linspace(obstacle_start[1], obstacle_end[1], num=n_pts)

        for (x_coord, y_coord) in zip(obstacle_x, obstacle_y):
            coords.append([x_coord, y_coord, 0])

        all_coords.append(coords)

    return all_coords

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
