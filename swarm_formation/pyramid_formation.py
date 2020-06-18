#!/usr/bin/env python

"""Square formation
"""

from math import sin, cos, pi, ceil
import rospy

from crazyflie_driver.msg import Position
from geometry_msgs.msg import Pose

from general_formation import FormationClass


class PyramidFormation(FormationClass):
    """Pyramid formation

    Notes:
        scale: Height of the pyramid
        swarm_pose: Same as position of CF 0

    Layouts:

        y
        |
        |
        |_____x

        ------ Top view -----
        2       3

            0

        1       4

        ------ Top view -----
        6               7

            2       3

                0

            1       4

        5               8

        ------ Side view -----
                0

            1       4

        5               8

    """
    def __init__(self, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        super(PyramidFormation, self).__init__(offset=offset)

        self.min_scale = 0.5

        # Attrs specific to square
        self.n_tier = 0 #: (int) Number of tier in the Pyramid
        self.tier_dist = 0 #: (float) Distance between each "tier" of the pyramid. Tier 0 at the top
        self.theta = 45*pi/180 #: (float) Angle between 1-0-4 (see side view)

    # Setter
    def set_n_cf(self, n):
        # Verify number of CFs, n-1 must be a multiple of 4
        if n > 0 and (n  - 1) % 4 == 0:
            self.n_cf = n
            self.n_cf_landed = 0
        else:
            self.n_cf_landed = (n  - 1) % 4
            self.n_cf = n - self.n_cf_landed
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" % self.n_cf_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_cf)
        self.land_extra_cf()

        self.n_tier = (self.n_cf - 1) / 4

        if self.n_tier != 0:
            self.tier_dist = self.scale/self.n_tier

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

    def compute_start_positions(self, landed=True):
        cf_num = 0
        center_x = self.scale * sin(self.theta)
        center_y = self.scale * cos(self.theta)
        center_z = self.scale

        # (dX, dY) sign for each position in tier
        tier_poses_sign = [(-1, -1), (-1, 1), (1, 1), (1, -1)]

        for i in range(self.n_cf):
            if rospy.is_shutdown():
                break
            start_goal = Position()
            start_goal.yaw = 0

            # Find tier information
            # i=0 -> tier=0, i=1,2,3,4 -> tier = 1, i=5,6,7,8 -> tier = 2 ...
            tier_num = ceil(i/4.0)
            tier_pos = i%4 # Position in the tier
            square_length = 2*sin(self.theta)*self.tier_dist*tier_num

            # Find goals

            z_pose = center_z - tier_num * self.tier_dist
            # To make sure starting position are at ground height
            if landed:
                start_goal.z = 0
            else:
                start_goal.z = z_pose

            x_dist = tier_poses_sign[tier_pos][0]*square_length/2
            y_dist = tier_poses_sign[tier_pos][1]*square_length/2

            start_goal.x = center_x + x_dist
            start_goal.y = center_y + y_dist
            self.cf_goals[cf_num] = start_goal

            # Find distances from center
            center_dist, theta, center_height =\
                self.compute_info_from_center([start_goal.x, start_goal.y, z_pose],
                                              [center_x, center_y, center_z])
            self.center_dist[cf_num] = center_dist
            self.angle[cf_num] = theta
            self.center_height[cf_num] = center_height

            cf_num += 1

        return self.cf_goals

    def update_scale(self):
        self.tier_dist = self.scale/self.n_tier # Space between CFs

        self.compute_start_positions(False)
