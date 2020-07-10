"""General tests
"""
import os
import numpy as np
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

# Read arguments from yaml file
PARENT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FILE_PATH = os.path.join(PARENT_DIR, 'conf.yaml')

with open(FILE_PATH) as f:
    YAML_CONF = yaml.load(f, Loader=yaml.FullLoader)

SOLVER_ARGS = YAML_CONF['trajectory_solver']

AGENT_ARGS = {'r_min': SOLVER_ARGS['r_min'],
              'col_radius_ratio': SOLVER_ARGS['col_radius_ratio'],
              'goal_thres': SOLVER_ARGS['goal_thres']}

def demo_two_agents():
    """Two agents trading spots
    """
    a_1 = Agent(AGENT_ARGS, start_pos=[0.5, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    a_2 = Agent(AGENT_ARGS, start_pos=[4.0, 2.0, 0.0], goal=[0.5, 2.0, 0.0])

    return [a_1, a_2]

def demo_wall():
    """Wall"""
    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    obs_coords = compute_obstacle([[(2.0, -1.0, 0.), (2.0, 2.5, 0.0)]], 15)

    return [a_1], obs_coords

def through_wall():
    """Wall"""
    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 2.0, 0.0], goal=[4.0, 2.0, 0.0])

    a_2 = Agent(AGENT_ARGS, start_pos=[5.0, 2.0, 0.0], goal=[0.0, 2.0, 0.0])

    obs_coords = compute_obstacle([[(2.0, -1.0, 0.), (2.0, 1.5, 0.0)],
                                   [(2.0, 2.5, 0.), (2.0, 5.0, 0.0)]], 30)

    return [a_1, a_2], obs_coords

def corners_2():
    """Two agents trading spots, starting from opposite corners
    """
    # a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    # a_2 = Agent(AGENT_ARGS, start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])

    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_2 = Agent(AGENT_ARGS, start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    return [a_1, a_2]

def corners_2_2():
    """Corners 2, v2
    """
    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(AGENT_ARGS, start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])

    return [a_1, a_2]

def corners_4():
    """Four agents, starting from opposite corners
    """
    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(AGENT_ARGS, start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(AGENT_ARGS, start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_4 = Agent(AGENT_ARGS, start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])

    return [a_1, a_2, a_3, a_4]

def six_agents():
    """Six agents
    """
    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 0.0, 0.0], goal=[1.5, 3.0, 0.0])
    a_2 = Agent(AGENT_ARGS, start_pos=[2.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(AGENT_ARGS, start_pos=[1.0, 2.5, 0.0], goal=[4.0, 0.0, 0.0])
    a_4 = Agent(AGENT_ARGS, start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])
    a_5 = Agent(AGENT_ARGS, start_pos=[2.5, 2.5, 0.0], goal=[4.0, 2.5, 0.0])
    a_6 = Agent(AGENT_ARGS, start_pos=[3.2, 3.2, 0.0], goal=[0.5, 0.0, 0.0])

    return [a_1, a_2, a_3, a_4, a_5, a_6]

def corners_6():
    """Six agents, starting from opposite corners
    """
    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 4.0, 0.0], goal=[4.0, 0.0, 0.0])
    a_2 = Agent(AGENT_ARGS, start_pos=[4.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(AGENT_ARGS, start_pos=[0.0, 0.0, 0.0], goal=[4.0, 4.0, 0.0])
    a_4 = Agent(AGENT_ARGS, start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])
    a_5 = Agent(AGENT_ARGS, start_pos=[2.0, 0.0, 0.0], goal=[2.0, 4.0, 0.0])
    a_6 = Agent(AGENT_ARGS, start_pos=[2.0, 4.0, 0.0], goal=[2.0, 0.0, 0.0])


    return [a_1, a_2, a_3, a_4, a_5, a_6]

def seven_agents():
    """Seven agents
    """
    a_1 = Agent(AGENT_ARGS, start_pos=[0.0, 0.0, 0.0], goal=[1.5, 3.0, 0.0])
    a_2 = Agent(AGENT_ARGS, start_pos=[2.0, 0.0, 0.0], goal=[0.0, 4.0, 0.0])
    a_3 = Agent(AGENT_ARGS, start_pos=[1.0, 2.5, 0.0], goal=[4.0, 0.0, 0.0])
    a_4 = Agent(AGENT_ARGS, start_pos=[4.0, 4.0, 0.0], goal=[0.0, 0.0, 0.0])
    a_5 = Agent(AGENT_ARGS, start_pos=[2.5, 2.5, 0.0], goal=[4.0, 2.5, 0.0])
    a_6 = Agent(AGENT_ARGS, start_pos=[3.2, 3.2, 0.0], goal=[0.5, 0.0, 0.0])

    a_7 = Agent(AGENT_ARGS, start_pos=[1.7, 0.5, 0.0], goal=[0.8, 3.8, 0.0])

    return [a_1, a_2, a_3, a_4, a_5, a_6, a_7]

def update_test():
    """To test when agents position are changed after solver initialization
    """
    a_list = ['a1', 'a2', 'a3']
    agents = {}
    for each_a in a_list:
        agents[each_a] = Agent(AGENT_ARGS, )

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
