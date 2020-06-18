#!/usr/bin/env python

"""Square formation
"""

import rospy
from crazyflie_driver.msg import Position

from general_formation import FormationClass


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
    def __init__(self, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        super(LineFormation, self).__init__(offset=offset)

        self.min_scale = 0.5

        self.cf_dist = 0

    # Setter
    def set_n_cf(self, n):
        # Verify number of CFs, all n are valid
        if n > 0:
            self.n_cf = n
            self.n_cf_landed = 0
        else:
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" % self.n_cf_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_cf)

        self.cf_dist = self.scale/(self.n_cf - 1) if self.n_cf > 1 else 0

    # Computing
    def compute_start_positions(self):
        cf_num = 0
        center_x = 0
        center_y = self.scale / 2
        center_z = 0

        for i in range(self.n_cf):
            if rospy.is_shutdown():
                break

            start_goal = Position()
            start_goal.yaw = 0
            z_dist = 0

            x_dist = 0
            y_dist = self.cf_dist*i

            start_goal.x = center_x + x_dist
            start_goal.y = y_dist
            start_goal.z = center_z + z_dist

            self.cf_goals[cf_num] = start_goal

            center_dist, theta, center_height =\
                self.compute_info_from_center([start_goal.x, start_goal.y, start_goal.z],
                                              [center_x, center_y, 0])
            self.center_dist[cf_num] = center_dist
            self.angle[cf_num] = theta
            self.center_height[cf_num] = center_height

            cf_num += 1

        return self.cf_goals

    def update_scale(self):
        self.cf_dist = self.scale / (self.n_cf - 1) if self.n_cf > 1 else 0
        self.compute_start_positions()
