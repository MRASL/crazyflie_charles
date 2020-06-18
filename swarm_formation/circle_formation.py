#!/usr/bin/env python

"""Square formation
"""

from math import sin, cos, pi
import rospy
from crazyflie_driver.msg import Position
from geometry_msgs.msg import Pose

from general_formation import FormationClass

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
    def __init__(self, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        super(CircleFormation, self).__init__(offset=offset)

        self.min_scale = 0.5

        self.angle_between_cf = 0 #: Angle between each CF (rad)

    # Setter
    def set_n_cf(self, n):
        # Verify number of CFs, all n are valid
        if n > 0:
            self.n_cf = n
            self.n_cf_landed = 0
        else:
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" % self.n_cf_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_cf)

        self.angle_between_cf = (2*pi)/(self.n_cf - 1)

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
        center_x = self.scale
        center_y = self.scale
        center_z = 0

        for i in range(self.n_cf):
            if rospy.is_shutdown():
                break

            start_goal = Position()
            start_goal.yaw = 0
            z_dist = 0

            if i == 0:
                x_dist = 0
                y_dist = 0
            else:
                angle = i*self.angle_between_cf
                x_dist = self.scale*cos(angle)
                y_dist = self.scale*sin(angle)

            start_goal.x = center_x + x_dist
            start_goal.y = center_y + y_dist
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
        self.compute_start_positions()
