#!/usr/bin/env python

"""Square formation
"""

from math import sqrt, floor
import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass, compute_info_from_center, R_MIN

class SquareFormation(FormationClass):
    """Square formation

    Notes:
        n_cf supported: Only perfect square (4, 9, 16...)
        scale: Length of a side

    Layouts:

        y
        |
        |
        |_____x

        1   3

        0   2

        ------

        2   5   8

        1   4   7

        0   3   6

    """
    def __init__(self):
        super(SquareFormation, self).__init__()

        # Attrs specific to square
        self.agents_per_side = 0 #: (float) Number of CF per side
        self.dist = 0 #: (float) Space between CFs

        self.compute_min_scale()

    # Setter
    def set_n_agents(self, n_agents):
        # Check if n is a perfect square
        n_sqrt = sqrt(n_agents)

        if n_sqrt - floor(n_sqrt) == 0 and n_agents > 0:
            self.n_agents_landed = 0
            self.n_agents = n_agents

        else:
            self.n_agents_landed = int(n_agents - floor(n_sqrt)**2)
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" %\
                self.n_agents_landed)
            self.n_agents = int(n_agents - self.n_agents_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_agents)

        self.find_extra_agents()
        self.update_formation_scale()
        self.compute_min_scale()

    # Computing
    def compute_min_scale(self):
        if self.n_agents > 1:
            self.min_scale = R_MIN*(self.agents_per_side - 1)
        else:
            self.min_scale = 0.0

    def compute_formation_positions(self):
        agent_num = 0

        center_offset = [self.scale/2, self.scale/2, 0]

        for i in range(self.agents_per_side):
            for j  in range(self.agents_per_side):
                if rospy.is_shutdown():
                    break

                # Initialize agent formation goal
                self.agents_goals[agent_num] = Position()

                # Formation position
                x_dist = i*self.dist - center_offset[0]
                y_dist = j*self.dist - center_offset[1]
                z_dist = 0

                # Information from center
                center_dist, theta, center_height =\
                    compute_info_from_center([x_dist, y_dist, z_dist])
                self.center_dist[agent_num] = center_dist
                self.angle[agent_num] = theta
                self.center_height[agent_num] = center_height

                agent_num += 1

    def update_formation_scale(self):
        self.agents_per_side = int(sqrt(self.n_agents)) # Number of CF per side

        # Space between CFs
        self.dist = self.scale/(self.agents_per_side-1) if self.agents_per_side > 1 else 0
