#!/usr/bin/env python

"""Square formation
"""

from math import sin, cos, pi
import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass, compute_info_from_center

class CircleFormation(FormationClass):
    """Circle formation

    Notes:
        n_cf supported: X
        scale: Radius of circle

    Layouts:

        y
        |
        |
        |_____x

            2

        3   0   1

            4

    """
    def __init__(self, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        super(CircleFormation, self).__init__(offset=offset)

        self.min_scale = 0.5

        self.angle_between_agents = 0 #: Angle between each agent (rad)

    # Setter
    def set_n_agents(self, n_agents):
        # Verify number of CFs, all n are valid
        if n_agents > 0:
            self.n_agents = n_agents
            self.n_agents_landed = 0
        else:
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" %\
                self.n_agents_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_agents)

        self.angle_between_agents = (2*pi)/(self.n_agents - 1)

    # Computing
    def compute_start_positions(self, formation_goal):
        center = [formation_goal.x,
                  formation_goal.y,
                  formation_goal.z]

        for i in range(self.n_agents):
            if rospy.is_shutdown():
                break

            agent_goal = Position()
            agent_goal.yaw = 0
            z_dist = 0

            if i == 0:
                x_dist = 0
                y_dist = 0
            else:
                angle = i*self.angle_between_agents
                x_dist = self.scale*cos(angle)
                y_dist = self.scale*sin(angle)

            agent_goal.x = center[0] + x_dist
            agent_goal.y = center[1] + y_dist
            agent_goal.z = center[2] + z_dist

            self.agents_goals[i] = agent_goal

            center_dist, theta, center_height =\
                compute_info_from_center([agent_goal.x, agent_goal.y, agent_goal.z],
                                         center)

            self.center_dist[i] = center_dist
            self.angle[i] = theta
            self.center_height[i] = center_height

        return self.agents_goals

    def update_scale(self, formation_goal):
        self.compute_start_positions(formation_goal)
