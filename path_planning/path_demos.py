#!/usr/bin/env python

"""Script to test path planning algorithm
"""

import time
import numpy as np
from path import Agent, TrajectorySolver

def demo():
    """Launch trajectories tests
    """
    start_time = time.time()

    # Choose demo to execute
    # agents, obstacles = demo_two_agents()
    # agents, obstacles = demo_wall()
    # agents, obstacles = corners_2()
    agents, obstacles = corners_4()
    # agents, obstacles = six_agents()
    # agents, obstacles = seven_agents()
    # agents, obstacles = nine_agents()

    solver = TrajectorySolver(agents)
    solver.set_obstacle(obstacles)

    solver.set_wait_for_input(False)
    solver.set_slow_rate(1.0)

    solver.solve_trajectories()

    print "Compute time:", (time.time() - start_time)*1000, "ms"

    solver.plot_trajectories()

def demo_two_agents():
    """Two agents trading spots
    """
    a_1 = Agent(start_pos=[0.0, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    a_2 = Agent(start_pos=[4.0, 2.0, 0.0], goal=[0.0, 2.0, 0.0])

    return [a_1, a_2], []

def demo_wall():
    """Wall"""
    a_1 = Agent(start_pos=[0.0, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    obs_coords = compute_obstacle([[2.0, 2.0, 0.], [2.0, 2.0, 0.0]], 1)

    return [a_1], obs_coords

def corners_2():
    """Two agents trading spots, starting from opposite corners
    """
    # a_1 = Agent(start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    # a_2 = Agent(start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])

    a_1 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_2 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    return [a_1, a_2], []

def corners_4():
    """Four agents, starting from opposite corners
    """
    a_1 = Agent(start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    return [a_1, a_2, a_3, a_4], []

def six_agents():
    """Six agents
    """
    a_1 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[1.5, 3.0, 0.0])
    a_2 = Agent(start_pos=[2.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[1.0, 2.5, 0.0], goal=[4.0, 0.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])
    a_5 = Agent(start_pos=[2.5, 2.5, 0.0], goal=[4.0, 2.5, 0.0])
    a_6 = Agent(start_pos=[3.2, 3.2, 0.0], goal=[0.5, 0.0, 0.0])

    return [a_1, a_2, a_3, a_4, a_5, a_6], []

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

    return [a_1, a_2, a_3, a_4, a_5, a_6, a_7], []

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

    return [a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8], []

def compute_obstacle(position, n_pts):
    """Compute coordinates of a wall

    Args:
        position (list of list): [obstacle_start, obstacle_end] , [x, y, z]
        n_pts (int): Number of pts

    Returns:
        list: All coords of wall
    """
    obstacle_start = position[0]
    obstacle_end = position[1]

    coords = []
    obstacle_x = np.linspace(obstacle_start[0], obstacle_end[0], num=n_pts)
    obstacle_y = np.linspace(obstacle_start[1], obstacle_end[1], num=n_pts)

    for (x_coord, y_coord) in zip(obstacle_x, obstacle_y):
        coords.append([x_coord, y_coord, 0])

    return coords

if __name__ == '__main__':
    demo()
