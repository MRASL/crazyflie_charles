#!/usr/bin/env python

"""Square formation
"""

from math import sqrt, floor
import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass

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
        n_cf_supported = []
        super(SquareFormation, self).__init__(n_cf_supported, offset=offset)

        self.min_scale = 0.5

        # Attrs specific to square
        self.cf_per_side = 0 #: (float) Number of CF per side
        self.dist = 0 #: (float) Space between CFs

    # Setter
    def set_n_cf(self, n):
        # Check if n is a perfect square
        n_sqrt = sqrt(n)

        if n_sqrt - floor(n_sqrt) == 0 and n > 0:
            self.n_cf_landed = 0
            self.n_cf = n

        else:
            self.n_cf_landed = int(n - floor(n_sqrt)**2)
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" % self.n_cf_landed)
            self.n_cf = int(n - self.n_cf_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_cf)

        self.land_extra_cf()

        self.cf_per_side = int(sqrt(self.n_cf)) # Number of CF per side

        # Space between CFs
        self.dist = self.scale/(self.cf_per_side-1) if self.cf_per_side > 1 else 0

    # Computing
    def compute_start_positions(self):
        cf_num = 0
        center_x = self.scale/2.0
        center_y = self.scale/2.0

        for i in range(self.cf_per_side):
            for j  in range(self.cf_per_side):
                if rospy.is_shutdown():
                    break

                start_goal = Position()
                start_goal.x = i*self.dist
                start_goal.y = j*self.dist
                start_goal.z = 0
                start_goal.yaw = 0
                self.cf_goals[cf_num] = start_goal

                center_dist, theta, center_height = \
                    self.compute_info_from_center([start_goal.x, start_goal.y, start_goal.z],
                                                  [center_x, center_y, 0])
                self.center_dist[cf_num] = center_dist
                self.angle[cf_num] = theta
                self.center_height[cf_num] = center_height

                cf_num += 1

        return self.cf_goals

    def update_scale(self):
         # Space between CFs
        self.dist = self.scale/(self.cf_per_side-1) if self.cf_per_side > 1 else 0
        self.compute_start_positions()
