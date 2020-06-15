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
    """Two agents trading spots, starting from opposite corners
    """
    a_1 = Agent(start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_4 = Agent(start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    return [a_1, a_2, a_3, a_4], []

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
