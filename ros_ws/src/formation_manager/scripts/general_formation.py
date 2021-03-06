#!/usr/bin/env python

"""Abstract Class that represents a general formation.

Cylindrical coordinates are used to specify the position of each agent in the formation.Center
of formation is defined as (0, 0, 0)
"""

from math import sin, cos, pi, sqrt, atan
import rospy

from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

class FormationClass(object):
    """Basic formation type

    Args:
        min_dist (:obj:`float`): Minimum distance between agents in formation
    """
    def __init__(self, min_dist):
        self._n_agents = 0 #: (int) Number of CF in the formation
        self._n_agents_landed = 0 #(int) Number of CF landed, not part of current formation
        self._min_dist = min_dist

        self._agents_goals = {} #: (dict of Position) Target Pose of all the CF
        self._center_dist = {} #: (dict of float) Keys: swarm id, Item: Distance from center
        self._angle = {} #: (dict of float) Keys: swarm id, Item: Angle(rad) from x axis

        self._extra_agents_id = [] #: list of int: Id of landed agents

        #: (dict of float) Keys: swarm id, Item: Height from center (<0 -> below swarm center)
        self._center_height = {}

        self._scale = 0.0 #: (float) scale of the formation
        self._min_scale = 0.0
        self._max_scale = 5.0

        self._min_height = 0.5 #: float: Minimum height of formation goal

    # General methods, valid between formations
    def get_agents_goals(self):
        """Return goal of each agent in formation

        Returns:
            dict of list: Keys: agents id, Items: goal [x, y, z]
        """
        goals = {}
        for agent_id, agent_goal in self._agents_goals.items():
            goals[agent_id] = [agent_goal.x, agent_goal.y, agent_goal.z]

        return goals

    def set_scale(self, new_scale):
        """Set scale of the formation

        Args:
            new_scale (float): New formation scale
        """
        self._scale = new_scale
        self._scale = self._min_scale if self._scale < self._min_scale else self._scale
        self._scale = self._max_scale if self._scale > self._max_scale else self._scale

        self.update_formation_scale()

        rospy.loginfo("Formation: Formation scale: %.2f" % self._scale)

    def update_agents_positions(self, formation_goal, crazyflies=None):
        """Compute goal of each agent and updates corresponding CF goal.

        Agent goal is calculated based on distance and angle of agent id from formation center.

        If crazyflies is not None, will update their goal.

        Args:
            formation_goal (Position): Goal of the formation
            crazyflies (dict, optional): Information of each Crazyflie
        """
        # Compute position of all CF
        for swarm_id in range(self._n_agents):
            if rospy.is_shutdown():
                break
            yaw = formation_goal.yaw

            # Could fail if swarm position is being calculated
            try:
                theta = self._angle[swarm_id] + yaw
            except KeyError:
                break

            x_dist = cos(theta) * self._center_dist[swarm_id]
            y_dist = sin(theta) * self._center_dist[swarm_id]
            z_dist = self._center_height[swarm_id]

            self._agents_goals[swarm_id].x = formation_goal.x + x_dist
            self._agents_goals[swarm_id].y = formation_goal.y + y_dist
            self._agents_goals[swarm_id].z = formation_goal.z + z_dist
            self._agents_goals[swarm_id].yaw = yaw

        # Update all CF formation goal based on swarm ID
        if crazyflies is not None:
            for _, cf_attrs in crazyflies.items():
                if rospy.is_shutdown():
                    break
                cf_id = cf_attrs["swarm_id"]
                try:
                    cf_attrs["formation_goal"].x = self._agents_goals[cf_id].x
                    cf_attrs["formation_goal"].y = self._agents_goals[cf_id].y
                    cf_attrs["formation_goal"].z = self._agents_goals[cf_id].z
                    cf_attrs["formation_goal"].yaw = self._agents_goals[cf_id].yaw
                except KeyError:
                    # Pass, keys arn't initialized yet because of new formation
                    pass

    def find_extra_agents(self):
        """Find id of agents in extra.

        Extra agents can happen when there are too many agents for a formation.
        i.e: Making a square with 10 agents will result in one extra agent.
        """
        self._extra_agents_id = []

        for agent_id in range(self._n_agents, self._n_agents + self._n_agents_landed + 1):
            self._extra_agents_id.append(agent_id)

    # Methods depending on formation
    def set_n_agents(self, n_agents):
        """Make sure there number of CF is supported by formation and sets it

        Args:
            n (int): Number of CF
        """
        if n_agents > 0:
            self._n_agents = n_agents
            rospy.loginfo("Formation made of %i crazyflies" % self._n_agents)
        else:
            self._n_agents = 0
            rospy.logerr("Unsuported number of CFs")

    def compute_min_scale(self):
        """Find minimum scale to make sure distance between agents is greater than min_dist
        """
        pass

    def compute_formation_positions(self):
        """Compute position of each agent from formation center

        Position are defined by radius from center (x, y plane), height from center and radius angle
        """
        pass

    def update_formation_scale(self):
        """Compute new formation information after the scale is changed.

        i.e: Distance/angle between agents

        Unique to each formation

        """
        pass

def compute_info_from_center(agent_position):
    """Calculate distance and angle from formation center

    Formation center is considered to be at 0, 0, 0

    Args:
        cf_position (list of float): Position from [0 , 0 ,0][x, y, z]

    Returns:
        list of float: [distance from center, angle from center, height from center]
    """
    x_dist = agent_position[0]
    y_dist = agent_position[1]
    z_dist = agent_position[2]

    center_dist = sqrt(x_dist**2 + y_dist**2)

    if x_dist != 0:
        theta = atan(y_dist/x_dist)
        if x_dist < 0 and y_dist < 0:
            theta = theta - pi
        elif x_dist < 0:
            theta = theta + pi

    else:
        if y_dist > 0:
            theta = pi/2
        else:
            theta = -pi/2

    return center_dist, theta, z_dist

def calculate_rot(start_orientation, rot):
    """Apply rotation to a quaternion

    Args:
        start_orientation (Quaternion): Initial orientation
        rot (Vector3): Angular speed to apply

    Returns:
        Quaternion: Result
    """
    rot_q = quaternion_from_euler(rot.x, rot.y, rot.z)
    orig_q = [start_orientation.x,
              start_orientation.y,
              start_orientation.z,
              start_orientation.w]
    res_q = quaternion_multiply(rot_q, orig_q)

    res_msg = Quaternion()
    res_msg.x = res_q[0]
    res_msg.y = res_q[1]
    res_msg.z = res_q[2]
    res_msg.w = res_q[3]

    return res_msg

def yaw_from_quat(quaternion):
    """Returns yaw from a quaternion

    Args:
        quaternion (Quaternion)

    Returns:
        float: Yaw
    """
    _, _, yaw = euler_from_quaternion([quaternion.x,
                                       quaternion.y,
                                       quaternion.z,
                                       quaternion.w])
    return yaw

def quat_from_yaw(yaw):
    """Compute a quaternion from yaw

    Pitch and roll are considered zero

    Args:
        yaw (float): Yaw

    Returns:
        Quaternion
    """
    x_quat, y_quat, z_quat, w_quat = quaternion_from_euler(0, 0, yaw)
    return Quaternion(x_quat, y_quat, z_quat, w_quat)
