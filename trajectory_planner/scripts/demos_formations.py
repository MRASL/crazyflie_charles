"""Formation demos
"""
# pylint: disable=invalid-name
# pylint: disable=wrong-import-position
# pylint: disable=import-error

import os
from math import sqrt, floor
import numpy as np
from numpy.linalg import norm
import pandas as pd
import yaml

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

# Read arguments from yaml file
PARENT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FILE_PATH = os.path.join(PARENT_DIR, 'conf.yaml')

with open(FILE_PATH) as f:
    YAML_CONF = yaml.load(f, Loader=yaml.FullLoader)

SOLVER_ARGS = YAML_CONF['trajectory_solver']

AGENT_ARGS = {'r_min': SOLVER_ARGS['r_min'],
              'col_radius_ratio': SOLVER_ARGS['col_radius_ratio'],
              'goal_thres': SOLVER_ARGS['goal_thres']}

def formation_demo(n_agents, formation_type):
    """Formation demo
    """
    formation = get_formation(formation_type)
    formation_start_pos = YAML_CONF['formation']['formation_start_pos']

    formation_goal = Position()
    formation_goal.x = formation_start_pos[0]
    formation_goal.y = formation_start_pos[1]
    formation_goal.z = formation_start_pos[2]
    formation_goal.yaw = formation_start_pos[3]

    formation.set_n_agents(n_agents)
    formation.set_scale(SCALE)
    formation.compute_formation_positions()
    formation.update_agents_positions(formation_goal)

    start_positions = compute_start_positions(n_agents)
    goals = {ag_id: [goal.x, goal.y, goal.z] for ag_id, goal in formation.agents_goals.items()}

    agent_list = []

    match_positions = link_agents_v2(start_positions, goals)

    for each_match in match_positions:
        agent = Agent(AGENT_ARGS, start_pos=each_match[0], goal=each_match[1])
        agent_list.append(agent)

    return agent_list

def get_formation(formation_type):
    """Init chosen formation

    Args:
        formation_type (str): Formation name

    Returns:
        Formation: Formation
    """
    formation_dist = YAML_CONF['formation']['formation_min_dist']

    if formation_type == "line":
        formation = LineFormation(formation_dist)

    elif formation_type == "square":
        formation = SquareFormation(formation_dist)

    elif formation_type == "v":
        formation = VFormation(formation_dist)

    elif formation_type == "pyramid":
        formation = PyramidFormation(formation_dist)

    elif formation_type == "circle":
        formation = CircleFormation(formation_dist)

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

def link_agents_v2(start_positions, goals):
    """To minimze total distance traveled when linking cf and formation agents

    Args:
        start_positions (dict): cf_id: position
        goals (dict): agent_id: goal

    Returns:
        list of tuple: (start_pos, goal)
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

    for agent_idx in range(n_goals): # col
        for goal_idx in range(n_goals): # row
            rows = slice(3*goal_idx, 3*goal_idx + 3)
            dist = norm(initial_position_mat[rows, agent_idx] - goal_mat[rows, 0])
            all_distances[goal_idx, agent_idx] = dist

    all_distances = pd.DataFrame(all_distances, index=agents_id_list, columns=cf_id_list)


    linked_goals = [] #: list of str: Ids of Agents (goals) that have been linked

    while len(linked_goals) < n_goals:
        close_cf = {} #: dict: Keys: Id of close cf, vals: list of (goal, goal_dist)

        # Find closest CF to each goal
        for agent_idx in all_distances.index:
            if agent_idx not in linked_goals:
                goal_dist = all_distances.loc[agent_idx].sort_values()

                closest_cf_id = goal_dist.index[0]
                try:
                    close_cf[closest_cf_id].append((agent_idx, goal_dist[closest_cf_id]))
                except KeyError:
                    close_cf[closest_cf_id] = [(agent_idx, goal_dist[closest_cf_id])]

        # Link each close CF to farthest goal
        for cf_id, close_goals in close_cf.items():
            goal_to_link = None
            max_dist = 0.0

            for each_close_goal in close_goals:
                dist = each_close_goal[1]

                if goal_to_link is None:
                    goal_to_link = each_close_goal[0]
                    max_dist = dist

                elif dist > max_dist:
                    goal_to_link = each_close_goal[0]
                    max_dist = dist

            all_distances = all_distances.drop(cf_id, axis='columns')
            linked_goals.append(goal_to_link)
            match_positions.append((start_positions[cf_id], goals[goal_to_link]))

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
