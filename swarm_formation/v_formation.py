#!/usr/bin/env python

"""Square formation
"""

from math import sin, cos, pi, ceil

import rospy
from crazyflie_driver.msg import Position
from geometry_msgs.msg import Pose

from general_formation import FormationClass


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
    def __init__(self, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        super(VFormation, self).__init__(offset=offset)

        self.min_scale = 0.5

        # Attrs specific to square
        #: (float) Number of CF per side. index 0 is right side, 1 is left side
        self.cf_per_side = [0, 0]
        self.dist = 0 #: (float) Space between CFs
        self.theta = 60*pi/180 #: (float) Opening of the formation (rad)

    # Setter
    def set_n_cf(self, n):
        # Verify number of CFs, all numbers are valid
        if n > 0:
            self.n_cf = n
            self.n_cf_landed = 0
        else:
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" % self.n_cf_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_cf)

        # Compute number of CF per side
        if self.n_cf % 2 != 0:
            self.cf_per_side[0] = (self.n_cf - 1) /2
            self.cf_per_side[1] = self.cf_per_side[0]

        else:
            self.cf_per_side[0] = self.n_cf / 2
            self.cf_per_side[1] = self.cf_per_side[0] - 1

        self.dist = self.scale/(self.cf_per_side[0]) # Space between CFs

    # Computing
    def compute_swarm_pose(self, crazyflie_list):
        """Compute pose of the swarm. Center is set at position of CF 0

        Args:
            crazyflie_list (dict of dict): Attrs of each CF

        Returns:
            Pose: Swarm Pose
        """

        # To simplify, swarm pose is the average of all the poses
        swarm_pose = Pose()

        for _, cf_attrs in crazyflie_list.items():
            if rospy.is_shutdown():
                break

            if cf_attrs["swarm_id"] == 0:
                pose = cf_attrs["pose"].pose

                swarm_pose.position = pose.position
                swarm_pose.orientation = pose.orientation

        return swarm_pose

    def compute_start_positions(self):
        cf_num = 0
        center_x = self.scale * cos(self.theta/2)
        center_y = self.scale * sin(self.theta/2)

        for i in range(self.n_cf):
            if rospy.is_shutdown():
                break

            start_goal = Position()
            start_goal.z = 0
            start_goal.yaw = 0

            # Find row number
            # i = 0 -> row = 0, i = 1 -> row = 1, i = 2 -> row = 1, i = 3 -> row = 2 ...
            row_num = ceil(i/2.0)

            # Find if above or below center
            sign = -1
            if i % 2 == 0:
                sign = 1

            x_dist = -self.dist*row_num * cos(self.theta/2)
            y_dist = self.dist*row_num * sin(self.theta/2) * sign

            start_goal.x = center_x + x_dist
            start_goal.y = center_y + y_dist
            self.cf_goals[cf_num] = start_goal

            center_dist, theta, center_height = \
                self.compute_info_from_center([start_goal.x, start_goal.y, 0],
                                              [center_x, center_y, 0])
            self.center_dist[cf_num] = center_dist
            self.angle[cf_num] = theta
            self.center_height[cf_num] = center_height

            cf_num += 1

        return self.cf_goals

    def update_scale(self):
        self.dist = self.scale/(self.cf_per_side[0]) # Space between CFs
        self.compute_start_positions()
