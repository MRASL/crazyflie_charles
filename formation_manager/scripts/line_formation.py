#!/usr/bin/env python

"""Line formation
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
            self._n_agents = n_agents
            self._n_agents_landed = 0
        else:
            rospy.loginfo("Formation: Unsuported number of agents, landing %i agents"\
                % self._n_agents_landed)

        rospy.loginfo("Formation: %i agents in formation" % self._n_agents)

        self.update_formation_scale()
        self.compute_min_scale()

    # Computing
    def compute_min_scale(self):
        if self._n_agents > 1:
            self._min_scale = self._min_dist*(self._n_agents - 1)
        else:
            self._min_scale = 0.0

    def compute_formation_positions(self):
        center_offset = self._scale/2

        for i in range(self._n_agents):
            if rospy.is_shutdown():
                break

            # Initialize agent formation goal
            self._agents_goals[i] = Position()

            # Compute formation position
            z_dist = 0
            x_dist = 0
            y_dist = self.agents_dist*i - center_offset

            # Compute information from center
            center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, z_dist])
            self._center_dist[i] = center_dist
            self._angle[i] = theta
            self._center_height[i] = center_height

    def update_formation_scale(self):
        self.agents_dist = self._scale / (self._n_agents - 1) if self._n_agents > 1 else 0
