#!/usr/bin/env python

"""Sinus formation
"""

from math import sin, pi
import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass, compute_info_from_center

class SinFormation(FormationClass):
    """Sinus formation

    Notes:
        scale: Total lenght of period
    """
    def __init__(self, min_dist):
        super(SinFormation, self).__init__(min_dist)

        self.agents_x_dist = 0 # [m]
        self.frequency = 0 # [rad]
        self.amplitude = 1 # [m]

        self.compute_min_scale()

    # Setter
    def set_n_agents(self, n_agents):
        # All numbers are valid
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
            x_dist = self.agents_x_dist*i - center_offset
            y_dist = self.amplitude*sin(self.frequency*x_dist)
            z_dist = 0

            # Compute information from center
            center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, z_dist])
            self._center_dist[i] = center_dist
            self._angle[i] = theta
            self._center_height[i] = center_height

        return self._agents_goals

    def update_formation_scale(self):
        self.agents_x_dist = self._scale / (self._n_agents - 1) if self._n_agents > 1 else 0

        self.frequency = (2*pi)/self._scale if self._scale > 0 else 0
