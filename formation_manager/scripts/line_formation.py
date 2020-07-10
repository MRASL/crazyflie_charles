#!/usr/bin/env python

"""Square formation
"""

import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass, compute_info_from_center


class LineFormation(FormationClass):
    """Line formation

    Notes:
        n_cf supported: X
        scale: Length of the line

    Layouts:

        y
        |
        |
        |_____x

        3

        2

        1

        0

    """
    def __init__(self, min_dist):
        super(LineFormation, self).__init__(min_dist)

        self.agents_dist = 0

        self.compute_min_scale()

    # Setter
    def set_n_agents(self, n_agents):
        # Verify number of CFs, all n are valid
        if n_agents > 0:
            self.n_agents = n_agents
            self.n_agents_landed = 0
        else:
            rospy.loginfo("Formation: Unsuported number of agents, landing %i agents"\
                % self.n_agents_landed)

        rospy.loginfo("Formation: %i agents in formation" % self.n_agents)

        self.update_formation_scale()
        self.compute_min_scale()

    # Computing
    def compute_min_scale(self):
        if self.n_agents > 1:
            self.min_scale = self.min_dis*(self.n_agents - 1)
        else:
            self.min_scale = 0.0

    def compute_formation_positions(self):
        center_offset = self.scale/2

        for i in range(self.n_agents):
            if rospy.is_shutdown():
                break

            # Initialize agent formation goal
            self.agents_goals[i] = Position()

            # Compute formation position
            z_dist = 0
            x_dist = 0
            y_dist = self.agents_dist*i - center_offset

            # Compute information from center
            center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, z_dist])
            self.center_dist[i] = center_dist
            self.angle[i] = theta
            self.center_height[i] = center_height

    def update_formation_scale(self):
        self.agents_dist = self.scale / (self.n_agents - 1) if self.n_agents > 1 else 0
