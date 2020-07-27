#!/usr/bin/env python

"""Module to compute the position of agents in a formation.

A formation is defined as a certain number of agents moving in a specific shape.

.. note::
    An agent is NOT a crazyflie. A crazyflie is the actual robot. An agent is a position in a
    formation

This package uses two classes:

    - :py:class:`FormationManager` to switch between formations
    - :py:class:`FormationClass` to compute the position of each agent for a formation.
      One for each formation.

To set a new formation, ``set_formation`` service is called. The new positions of agents will be
computed using the desired :py:class:`FormationClass` child class.

Based on the position of each crazyflie, :py:class:`FormationManager` will then link each CF to an
agent. Once a CF is linked to an agent, this CF will follow this agent's position in the formation.

``FormationManager`` class
--------------------------
"""
from math import sin, cos, pi
import ast
import numpy as np
from numpy.linalg import norm
import pandas as pd

import rospy
from geometry_msgs.msg import Pose, Twist
from crazyflie_driver.msg import Position
from std_srvs.srv import Empty
from formation_manager.srv import SetFormation, GetFormationList

from square_formation import SquareFormation
from line_formation import LineFormation
from circle_formation import CircleFormation
from pyramid_formation import PyramidFormation
from v_formation import VFormation

class FormationManager(object):
    """To change between formations and associate agents and CFs
    """
    def __init__(self, cf_list, min_dist, start_goal):
        self._min_dist = min_dist #: (float) Minimum distance between agents in formation
        self._cf_list = cf_list
        self._n_cf = len(cf_list) #: (int) Number of CF in the swarm
        self._pose_cnt = 0 #: (int) To know when compute pose

        #: In abs ctrl mode, moves in world/ In rel ctrl mode, moves relative to yaw
        self.abs_ctrl_mode = False

        self._rate = rospy.Rate(100)

        _initial_formation_goal = Position() #: Position: formation start position
        _initial_formation_goal.x = start_goal[0]
        _initial_formation_goal.y = start_goal[1]
        _initial_formation_goal.z = start_goal[2]
        _initial_formation_goal.yaw = start_goal[3]

        self.scale = 1.0

        #: All possible formations
        self._formations = {"square": SquareFormation(self._min_dist),
                            "v": VFormation(self._min_dist),
                            "pyramid": PyramidFormation(self._min_dist),
                            "circle": CircleFormation(self._min_dist),
                            "line": LineFormation(self._min_dist),}
        self.formation = None #: (str) Current formation

        #: (list of list of float): Starting pos of each agent in formation, independant of CF id
        self.start_positions = []

        self.extra_agents = []

        # Publisher
        self._formation_pose_pub = rospy.Publisher('/formation_pose', Pose, queue_size=1)
        self._formation_goal_pub = rospy.Publisher('/formation_goal', Position, queue_size=1)
        self._formation_pose = Pose()
        self._formation_goal = Position()
        self._formation_goal_vel = Twist()

        self._formation_goal = _initial_formation_goal

        # Subscribers
        rospy.Subscriber("/formation_goal_vel", Twist, self._formation_goal_vel_handler)

        self.crazyflies = {} #: dict of list: Information of each CF
        # Initialize each CF
        for cf_id in cf_list:
            self.crazyflies[cf_id] = {"formation_goal": Position(), # msg
                                      "swarm_id": 0,                # int, id in the swarm
                                      "formation_goal_pub": None,   # publisher
                                      "initial_position": None}

            # Add goal publisher
            self.crazyflies[cf_id]["formation_goal_pub"] =\
                rospy.Publisher('/%s/formation_goal' % cf_id, Position, queue_size=1)

        # Start services
        rospy.Service('/set_formation', SetFormation, self._set_formation)
        rospy.Service('/toggle_ctrl_mode', Empty, self._toggle_ctrl_mode)
        rospy.Service('/formation_inc_scale', Empty, self._formation_inc_scale)
        rospy.Service('/formation_dec_scale', Empty, self._formation_dec_scale)
        rospy.Service('/get_formations_list', GetFormationList, self._return_formation_list)

    # Services and subscriptions
    def _formation_goal_vel_handler(self, goal_vel):
        """To change formation goal based on a velocity

        Depends on ctrl mode

        Args:
            goal_vel (Twist): Swarm goal velocity
        """
        self._formation_goal_vel = goal_vel

        # Moves relative to world
        if self.abs_ctrl_mode:
            self._formation_goal.x += self._formation_goal_vel.linear.x
            self._formation_goal.y += self._formation_goal_vel.linear.y
            self._formation_goal.z += self._formation_goal_vel.linear.z
            self._formation_goal.yaw += self._formation_goal_vel.angular.z

        # Moves relative to orientation. X axis in front, y axis on toward the left, z axis up
        else:
            x_vel = self._formation_goal_vel.linear.x
            y_vel = self._formation_goal_vel.linear.y
            theta = self._formation_goal.yaw
            x_dist = x_vel * cos(theta) + y_vel * cos(theta + pi/2.0)
            y_dist = x_vel * sin(theta) + y_vel * sin(theta + pi/2.0)
            self._formation_goal.x += x_dist
            self._formation_goal.y += y_dist
            self._formation_goal.yaw += self._formation_goal_vel.angular.z

            # Make sure formation stays above ground
            new_z = self._formation_goal.z + self._formation_goal_vel.linear.z
            if new_z > self.formation.min_height:
                self._formation_goal.z = new_z
            else:
                self._formation_goal.z = self._formation_goal.z

        # Update formation goal of each CF
        if self.formation is not None:
            self.formation.update_agents_positions(self._formation_goal, self.crazyflies)

    def _set_formation(self, srv_call):
        """Set formation

        Args:
            srv_call (SetFormation): Formation to set

        Returns:
            bool: success
        """
        new_formation = srv_call.formation
        cf_initial_positions = ast.literal_eval(srv_call.positions)
        new_goal = ast.literal_eval(srv_call.goal)

        valid_formation = True

        if new_formation in self._formations.keys():
            rospy.loginfo("Formation: Setting formation to %s" % new_formation)

            if new_goal is not None:
                self._formation_goal.x = new_goal[0]
                self._formation_goal.y = new_goal[1]
                self._formation_goal.z = new_goal[2]
                self._formation_goal.yaw = new_goal[3]

            self.formation = self._formations[new_formation]
            self.init_formation(cf_initial_positions)
            self.link_swarm_and_formation()
            self.formation.update_agents_positions(self._formation_goal, self.crazyflies)

        else:
            rospy.logerr("Formation: Invalid formation: %s" % new_formation)
            valid_formation = False

        return {"success": valid_formation, "extra_cf": ','.join(self.extra_agents)}

    def _toggle_ctrl_mode(self, _):
        """Toggle control mode

        Absolute mode: Controls in world reference

        Relative mode: Controls depending of swarm orientation
        """
        self.abs_ctrl_mode = not self.abs_ctrl_mode
        if self.abs_ctrl_mode:
            rospy.loginfo("Formation: Control mode set to absolute")
        else:
            rospy.loginfo("Formation: Control mode set to relative")

        return {}

    def _formation_inc_scale(self, _):
        """Service to increase scale of the formation
        """
        self.scale += 0.5

        self.formation.set_scale(self.scale)
        self.check_goal_height()

        # Find new agents positions around goal
        self.formation.compute_formation_positions()

        # Update CFs positions
        self.formation.update_agents_positions(self._formation_goal, self.crazyflies)

        return {}

    def _formation_dec_scale(self, _):
        """Service to reduce scale of the formation
        """
        self.scale -= 0.5

        self.formation.set_scale(self.scale)
        self.check_goal_height()

        # Find new agents positions around goal
        self.formation.compute_formation_positions()

        # Update CFs positions
        self.formation.update_agents_positions(self._formation_goal, self.crazyflies)

        return {}

    def _return_formation_list(self, _):
        """To get a list of all possible formations

        Returns:
            list of str: Possible formation
        """
        possible_formations = self._formations.keys()
        return {"formations": ','.join(possible_formations)}

    # Formation initialization methods
    def init_formation(self, initial_positions):
        """Initialize formation goal and cf positions

        Args:
            initial_positions (dict of list): Keys: Id of CF, Items: Initial position [x, y, z]
        """
        self.formation.set_n_agents(len(self._cf_list))
        self.formation.set_scale(self.scale)

        self.check_goal_height()

        self.formation.compute_formation_positions()
        self.formation.update_agents_positions(self._formation_goal)

        for cf_id, initial_position in initial_positions.items():
            self.crazyflies[cf_id]["initial_position"] = initial_position

    def check_goal_height(self):
        """Make sure formation goal is above formation minimum height.

        If it's not the case, set formation goal height to mimimum
        """
        if self.formation.min_height > self._formation_goal.z:
            self._formation_goal.z = self.formation.min_height

    def link_swarm_and_formation(self):
        """Link each agent of formation to a CF of the swarm and initialize formation goals
        """
        agents_id_list, goal_mat = self.create_goal_matrix()
        n_goals = len(agents_id_list)

        cf_id_list, initial_position_mat = self.create_initial_pos_matrix(n_goals)

        all_distances = self.compute_distances(cf_id_list, initial_position_mat,
                                               agents_id_list, goal_mat)

        match_positions = self.find_association(all_distances)

        self.update_associations(cf_id_list, match_positions)

    def create_goal_matrix(self):
        """Create a matrix with all the goals stacked vertically

        Returns:
            tuple: (Agent id corresponding to each row, goal matrix)
        """
        agents_goals = self.formation.get_agents_goals()
        agents_id_list = []
        goal_mat = None

        for agent_id, agent_goal in agents_goals.items():
            agents_id_list.append(agent_id)
            if goal_mat is None:
                goal_mat = np.array([agent_goal]).reshape(3, 1)
            else:
                goal_mat = np.vstack((goal_mat, np.array(agent_goal).reshape(3, 1)))

        return agents_id_list, goal_mat

    def create_initial_pos_matrix(self, n_goals):
        """Create initial position matrix: cols Initial position of cf, rows: for each goal

        Returns:
            tuple: (List of cf ids, initial position matrix)
        """
        initial_position_mat = None
        cf_id_list = [] # list of str: Cf id corresponding to each col
        for cf_id, cf_vals in self.crazyflies.items():
            cf_id_list.append(cf_id)
            initial_pos = np.array([cf_vals["initial_position"]]).reshape(3, 1)
            current_pos = initial_pos

            for _ in range(1, n_goals):
                current_pos = np.vstack((current_pos, initial_pos))

            if initial_position_mat is None:
                initial_position_mat = current_pos
            else:
                initial_position_mat = np.hstack((initial_position_mat, current_pos))

        return cf_id_list, initial_position_mat

    def compute_distances(self, cf_id_list, initial_position_mat, agents_id_list, goal_mat):
        """Compute distance from each goal and CF

        Args:
            cf_id_list (list): Ids of all CF in swarm
            initial_position_mat (arrray): Initial position matrix
            agents_id_list (list): Ids of all agents in formation
            goal_mat (array): Goal matrix

        Returns:
            pd.DataFrame: Index: Agent id, Cols: CF id, vals: Distance
        """
        n_goals = len(agents_id_list)

        # Find distances
        all_distances = np.zeros((n_goals, self._n_cf))

        for cf_idx in range(self._n_cf): # col
            for goal_idx in range(n_goals): # row
                rows = slice(3*goal_idx, 3*goal_idx + 3)
                dist = norm(initial_position_mat[rows, cf_idx] - goal_mat[rows, 0])
                all_distances[goal_idx, cf_idx] = dist

        all_distances = pd.DataFrame(all_distances, index=agents_id_list, columns=cf_id_list)
        return all_distances

    def find_association(self, all_distances):
        """To minimze total distance traveled when linking cf and formation agents
        """
        n_goals = len(all_distances.index)
        linked_goals = [] #: list of str: Ids of Agents (goals) that have been linked
        match_positions = []

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
                match_positions.append((cf_id, goal_to_link))

        return match_positions

    def update_associations(self, cf_id_list, match_positions):
        """Update CF associations and extra agents list
        """
        linked_cf = []

        for each_match in match_positions:
            cf_id = each_match[0]
            agent_id = each_match[1]
            self.crazyflies[cf_id]['swarm_id'] = agent_id
            linked_cf.append(cf_id)

        self.extra_agents = [cf_id for cf_id in cf_id_list if cf_id not in linked_cf]

    # Publishers
    def _publish_cf_formation_goal(self):
        """Publish formation goal of each CF
        """
        for _, cf_attrs in self.crazyflies.items():
            if rospy.is_shutdown():
                break
            cf_attrs["formation_goal_pub"].publish(cf_attrs["formation_goal"])

    def _publish_formation_pose(self):
        """Publish current position of the formation (center)
        """
        self._formation_pose_pub.publish(self._formation_pose)

    def _publish_formation_goal(self):
        """Publish current goal of the formation
        """
        self._formation_goal_pub.publish(self._formation_goal)

    def run_formation(self):
        """Execute formation
        """
        while not rospy.is_shutdown():
            if self.formation is not None:
                self._publish_cf_formation_goal()
                self._publish_formation_pose()
                self._publish_formation_goal()
            self._rate.sleep()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarm_formation_manager', anonymous=False)
    rospy.loginfo('Formation: Initialization...')

    # Get params
    while True: # Make sure cf_list has been set by `swarm_controller`
        try:
            CF_LIST = rospy.get_param("cf_list")
            break
        except KeyError:
            pass

    MIN_DIST = rospy.get_param("formation")["formation_min_dist"]
    START_GOAL = rospy.get_param("formation")["formation_start_pos"]

    # Initialize swarm
    FORMATION_MANAGER = FormationManager(CF_LIST, MIN_DIST, START_GOAL)

    FORMATION_MANAGER.run_formation()

    rospy.spin()
