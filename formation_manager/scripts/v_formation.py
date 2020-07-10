#!/usr/bin/env python

"""Square formation
"""

from math import sin, cos, pi, ceil

import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass, compute_info_from_center


class VFormation(FormationClass):
    """V formation

    Args:
        FormationType ([type]):

    Notes:
        scale: Length of one side
        swarm_pose: Same as position of CF 0

    Layouts:

        y
        |
        |
        |_____x

        2
            0
        1

        ------

            2
                0
            1
        3

        ------

        4
            2
                0
            1
        3

    """
    def __init__(self, min_dist):
        super(VFormation, self).__init__(min_dist)

        # Attrs specific to square
        #: (float) Number of CF per side. index 0 is right side, 1 is left side
        self.agents_per_side = [0, 0]
        self.dist = 0 #: (float) Space between CFs
        self.theta = 60*pi/180 #: (float) Opening of the formation (rad)

        self.compute_min_scale()

    # Setter
    def set_n_agents(self, n_agents):
        # Verify number of Agents, all numbers are valid
        if n_agents > 0:
            self.n_agents = n_agents
            self.n_agents_landed = 0
        else:
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" %\
                self.n_agents_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_agents)

        # Compute number of agents per side
        if self.n_agents % 2 != 0:
            self.agents_per_side[0] = (self.n_agents - 1) /2
            self.agents_per_side[1] = self.agents_per_side[0]

        else:
            self.agents_per_side[0] = self.n_agents / 2
            self.agents_per_side[1] = self.agents_per_side[0] - 1

        self.update_formation_scale()
        self.compute_min_scale()

    # Computing
    def compute_min_scale(self):
        self.min_scale = self.min_dist * self.agents_per_side[0]

    def compute_formation_positions(self):
        for i in range(self.n_agents):
            if rospy.is_shutdown():
                break

            # Initialize agent formation goal
            self.agents_goals[i] = Position()

            # Find row number
            # i = 0 -> row = 0, i = 1 -> row = 1, i = 2 -> row = 1, i = 3 -> row = 2 ...
            row_num = ceil(i/2.0)

            # Find if above or below center
            sign = -1
            if i % 2 == 0:
                sign = 1

            x_dist = -self.dist*row_num * cos(self.theta/2)
            y_dist = self.dist*row_num * sin(self.theta/2) * sign

            center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, 0])
            self.center_dist[i] = center_dist
            self.angle[i] = theta
            self.center_height[i] = center_height

    def update_formation_scale(self):
        self.dist = self.scale/(self.agents_per_side[0]) # Space between agents
