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
    def __init__(self, min_dist):
        super(CircleFormation, self).__init__(min_dist)

        self.angle_between_agents = 0 #: Angle between each agent (rad)
        self.compute_min_scale()

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

        self.update_formation_scale()
        self.compute_min_scale()

    # Computing
    def compute_min_scale(self):
        # Radius greater than R_MIN and distance between agents on circle greater than R_MIN
        self.min_scale = 0.0
        if self.angle_between_agents > 0:
            # Find scale when agents on circle are at 0.35m
            self.min_scale = self.min_dist/(2*sin(self.angle_between_agents/2))

            # Scale is smallest radius if theta is to big
            self.min_scale = self.min_dist if self.min_scale < self.min_dist else self.min_scale

    def compute_formation_positions(self):
        for i in range(self.n_agents):
            if rospy.is_shutdown():
                break

            # Initialize agent formation goal
            self.agents_goals[i] = Position()

            # Compute formation position
            z_dist = 0
            if i == 0:
                x_dist = 0
                y_dist = 0
            else:
                angle = i*self.angle_between_agents
                x_dist = self.scale*cos(angle)
                y_dist = self.scale*sin(angle)

            # Compute information from center
            center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, z_dist])
            self.center_dist[i] = center_dist
            self.angle[i] = theta
            self.center_height[i] = center_height

        return self.agents_goals

    def update_formation_scale(self):
        self.angle_between_agents = (2*pi)/(self.n_agents - 1) if self.n_agents > 1 else 0
