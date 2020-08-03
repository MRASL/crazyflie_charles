#!/usr/bin/env python

"""Generate random start and goal for each CF
"""

# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import

import os
import sys
import random as rand
import numpy as np
from numpy import array
from numpy.linalg import norm
import yaml

# pylint: disable=invalid-name
# pylint: disable=import-error
# pylint: disable=wrong-import-position
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
traj_path = os.path.join(parentdir, 'scripts')
os.sys.path.insert(0, traj_path)

from agent import Agent
# pylint: enable=invalid-name
# pylint: enable=import-error
# pylint: enable=wrong-import-position

# Read arguments from yaml file
PARENT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FILE_PATH = os.path.join(PARENT_DIR, 'swarm_manager/conf/swarm_conf.yaml')

with open(FILE_PATH) as f:
    YAML_CONF = yaml.load(f, Loader=yaml.FullLoader)

SOLVER_ARGS = YAML_CONF['trajectory_solver']

AGENT_ARGS = {'r_min': SOLVER_ARGS['r_min'],
              'col_radius_ratio': SOLVER_ARGS['col_radius_ratio'],
              'goal_thres': SOLVER_ARGS['goal_thres']}

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
    # print "Seed used:", seed

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
