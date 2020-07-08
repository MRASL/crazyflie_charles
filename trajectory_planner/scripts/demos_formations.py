"""Formation demos
"""
# pylint: disable=invalid-name
# pylint: disable=wrong-import-position
# pylint: disable=import-error

import os
import numpy as np
from numpy.linalg import norm
import pandas as pd
from math import sqrt, floor

from crazyflie_driver.msg import Position
from agent import Agent

parentdir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
formation_path = os.path.join(parentdir, 'formation_manager/scripts')
os.sys.path.insert(0, formation_path)

from line_formation import LineFormation
from square_formation import SquareFormation
from circle_formation import CircleFormation
from pyramid_formation import PyramidFormation
from v_formation import VFormation

START_DIST = 0.5
SCALE = 1.0

def formation_demo(n_agents, formation_type):
    """Formation demo
    """
    formation = get_formation(formation_type)

    formation_goal = Position()
    formation_goal.x = 2.5
    formation_goal.y = 2.5
    formation_goal.z = 0.0


    formation.set_n_agents(n_agents)
    formation.set_scale(SCALE)
    formation.compute_formation_positions()
    formation.update_agents_positions(formation_goal)

    start_positions = compute_start_positions(n_agents)
    goals = {ag_id: [goal.x, goal.y, goal.z] for ag_id, goal in formation.agents_goals.items()}

    agent_list = []

    match_positions = link_agents(start_positions, goals)

    for each_match in match_positions:
        agent = Agent(start_pos=each_match[0], goal=each_match[1])
        agent_list.append(agent)

    return agent_list

def get_formation(formation_type):
    """Init chosen formation

    Args:
        formation_type (str): Formation name

    Returns:
        Formation: Formation
    """
    if formation_type == "line":
        formation = LineFormation()

    elif formation_type == "square":
        formation = SquareFormation()

    elif formation_type == "v":
        formation = VFormation()

    elif formation_type == "pyramid":
        formation = PyramidFormation()

    elif formation_type == "circle":
        formation = CircleFormation()

    else:
        print "UNKNOWN FORMATION"
        formation = None

    return formation

def link_agents(start_positions, goals):
    """To minimze total distance traveled when linking cf and formation agents
    """
    match_positions = []

    # Create goal matrix
    agents_goals = goals
    agents_id_list = [] # list of int: Agent id corresponding to each row
    goal_mat = None
    for agent_id, agent_goal in agents_goals.items():
        agents_id_list.append(agent_id)
        if goal_mat is None:
            goal_mat = np.array([agent_goal]).reshape(3, 1)
        else:
            goal_mat = np.vstack((goal_mat, np.array(agent_goal).reshape(3, 1)))
    n_goals = len(agents_id_list)

    # Create initial position matrix: cols Initial position of cf, rows: for each goal
    initial_position_mat = None
    cf_id_list = [] # list of str: Cf id corresponding to each col
    for cf_id, start_pos in start_positions.items():
        cf_id_list.append(cf_id)
        initial_pos = np.array([start_pos]).reshape(3, 1)
        current_pos = initial_pos

        for _ in range(1, n_goals):
            current_pos = np.vstack((current_pos, initial_pos))

        if initial_position_mat is None:
            initial_position_mat = current_pos
        else:
            initial_position_mat = np.hstack((initial_position_mat, current_pos))


    # Find distances
    all_distances = np.zeros((n_goals, n_goals))

    for cf_idx in range(n_goals): # col
        for goal_idx in range(n_goals): # row
            rows = slice(3*goal_idx, 3*goal_idx + 3)
            dist = norm(initial_position_mat[rows, cf_idx] - goal_mat[rows, 0])
            all_distances[goal_idx, cf_idx] = dist

    all_distances = pd.DataFrame(all_distances, index=agents_id_list, columns=cf_id_list)

    # Find closest CF to each goal
    linked_cf = [] #: list of str: Ids of CF that have been linked
    for cf_idx in all_distances.index:
        goal_dist = all_distances.loc[cf_idx].sort_values()

        sorted_cf = goal_dist.index

        for each_cf in sorted_cf:
            if each_cf not in linked_cf:
                linked_cf.append(each_cf)
                match_positions.append((start_positions[each_cf], goals[cf_idx]))
                break

    return match_positions

def compute_start_positions(n_agents):
    """Compute start positions

    Args:
        n_agents (int): Number of agents

    Returns:
        list of list: Position of each agent
    """
    sq_length = int(floor(sqrt(n_agents)))
    n_extra = n_agents - sq_length**2
    start_positions = {}
    current_pos = 0

    for i in range(sq_length):
        for j in range(sq_length):
            start_positions[current_pos] = [i*START_DIST, j*START_DIST, 0.0]
            current_pos += 1

    for i in range(n_extra):
        start_positions[current_pos] = [i*START_DIST, sq_length*START_DIST, 0.0]
        current_pos += 1


    return start_positions
