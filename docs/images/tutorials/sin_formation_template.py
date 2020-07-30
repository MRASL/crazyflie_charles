#!/usr/bin/env python

"""Sinus formation
"""

from math import sin, cos, pi
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

        # TODO: Add formation specific attributes

        self.compute_min_scale()

    # Setter
    def set_n_agents(self, n_agents):

        # TODO: Verify number of agents is valid

        self.update_formation_scale()
        self.compute_min_scale()

    # Computing
    def compute_min_scale(self):
        # TODO: Compute min scale where min distance between agents is R_MIN
        pass

    def compute_formation_positions(self):
        for i in range(self._n_agents):
            if rospy.is_shutdown():
                break

            # Initialize agent formation goal
            self._agents_goals[i] = Position()

            # Compute formation position
            # TODO: Compute agent position from center

            # Compute information from center
            center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, z_dist])
            self._center_dist[i] = center_dist
            self._angle[i] = theta
            self._center_height[i] = center_height

        return self._agents_goals

    def update_formation_scale(self):
        #TODO: Update formation scale
        pass
