#!/usr/bin/env python

"""Square formation
"""

from math import sqrt, floor
import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass, compute_info_from_center

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
    def __init__(self, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        super(SquareFormation, self).__init__(offset=offset)

        self.min_scale = 0.5

        # Attrs specific to square
        self.agents_per_side = 0 #: (float) Number of CF per side
        self.dist = 0 #: (float) Space between CFs

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

        self.land_extra_agents()

        self.agents_per_side = int(sqrt(self.n_agents)) # Number of CF per side

        # Space between CFs
        self.dist = self.scale/(self.agents_per_side-1) if self.agents_per_side > 1 else 0

    # Computing
    def compute_start_positions(self, formation_goal):
        agent_num = 0
        center = [formation_goal.x,
                  formation_goal.y,
                  formation_goal.z]

        center_offset = [self.scale/2, self.scale/2, 0]

        for i in range(self.agents_per_side):
            for j  in range(self.agents_per_side):
                if rospy.is_shutdown():
                    break

                start_goal = Position()
                start_goal.x = i*self.dist + center[0] - center_offset[0]
                start_goal.y = j*self.dist + center[1] - center_offset[1]
                start_goal.z = center[2]
                start_goal.yaw = 0
                self.agents_goals[agent_num] = start_goal

                center_dist, theta, center_height = \
                    compute_info_from_center([start_goal.x, start_goal.y, start_goal.z],
                                             center)
                self.center_dist[agent_num] = center_dist
                self.angle[agent_num] = theta
                self.center_height[agent_num] = center_height

                agent_num += 1

        return self.agents_goals

    def update_scale(self, formation_goal):
         # Space between CFs
        self.dist = self.scale/(self.agents_per_side-1) if self.agents_per_side > 1 else 0
        self.compute_start_positions(formation_goal)
