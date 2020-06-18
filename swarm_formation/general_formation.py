#!/usr/bin/env python

"""Abstract Class that represents a general formation"""

from math import sin, cos, pi, sqrt, atan
import rospy
import numpy as np

from geometry_msgs.msg import Pose, Quaternion
from crazyflie_driver.msg import Position
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion


class FormationClass(object):
    """Basic formation type

    """

    def __init__(self, n_cf_supported=None, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        self.n_cf = 0 #: (int) Number of CF in the formation
        self.n_cf_landed = 0 #(int) Number of CF landed, not part of current formation

        self.cf_goals = {} #: (dict of Position) Target Pose of all the CF
        self.center_dist = {} #: (dict of float) Keys: swarm id, Item: Distance from center
        self.angle = {} #: (dict of float) Keys: swarm id, Item: Angle(rad) from x axis

        #: (dict of float) Keys: swarm id, Item: Height from center (<0 -> below swarm center)
        self.center_height = {}

        #: (list of int) Different number of CF possible for each formation
        self.n_cf_supported = n_cf_supported

        self.initial_offset = Pose() #: (Pose): Offset of the center of the formation from 0,0,0
        self.initial_offset.position.x = offset[0]
        self.initial_offset.position.y = offset[1]
        self.initial_offset.position.z = offset[2]

        self.scale = 1.0 #: (float) scale of the formation
        self.min_scale = 0
        self.max_scale = 5

    # General methods, valid between formations
    def get_n_cf(self):
        """Returns number of CF in the swarm

        Returns:
            int: Number of CF in the swarm
        """
        return self.n_cf

    def set_offset(self, x_offset, y_offset, z_offset):
        """Set starting offset of swarm position

        Args:
            x_offset (float): x offset
            y_offset (float): y offset
            z_offset (float): z offset
        """
        self.initial_offset.position.x = x_offset
        self.initial_offset.position.y = y_offset
        self.initial_offset.position.z = z_offset

    def change_scale(self, to_inc):
        """Change scale of formation.

        If to_inc is True, increase scale. Else, decrease it.

        Note:
            Scale is unique to each formation. i.e: Scale of circle is the radius, scale of square
            is a side length

        Args:
            to_inc ([type]): [description]
        """
        if to_inc:
            self.scale += 0.5

        else:
            self.scale -= 0.5

        self.scale = self.min_scale if self.scale < self.min_scale else self.scale
        self.scale = self.max_scale if self.scale > self.max_scale else self.scale

        self.update_scale()

    def compute_info_from_center(self, cf_position, formation_center):
        """Calculate distance and angle from formation center

        Args:
            cf_position (list of float): Position [x, y, z]
            formation_center (list of float): Center position [x, y, z]

        Returns:
            list of float: [distance from center, angle from center, height from center]
        """
        x_dist = cf_position[0] - formation_center[0]
        y_dist = cf_position[1] - formation_center[1]
        z_dist = cf_position[2] - formation_center[2]

        center_dist = sqrt(x_dist**2 + y_dist**2)

        if x_dist != 0:
            theta = atan(y_dist/x_dist)
            if x_dist < 0 and y_dist < 0:
                theta = theta - pi
            elif x_dist < 0:
                theta = theta + pi

        else:
            if y_dist > 0:
                theta = pi/2
            else:
                theta = -pi/2

        return center_dist, theta, z_dist

    def compute_cf_goals(self, crazyflies, swarm_goal):
        """Compute goal of each crazyflie based on target position of the swarm

        Args:
            crazyflies (dict): Information of each Crazyflie
            swarm_goal (Position): Goal of the swarm
        """
        # Compute position of all the CF

        for swarm_id in range(self.n_cf):
            if rospy.is_shutdown():
                break
            yaw = swarm_goal.yaw

            # Could fail if swarm position is being calculated
            try:
                theta = self.angle[swarm_id] + yaw
            except KeyError:
                break

            x_dist = cos(theta) * self.center_dist[swarm_id]
            y_dist = sin(theta) * self.center_dist[swarm_id]
            z_dist = self.center_height[swarm_id]

            self.cf_goals[swarm_id].x = swarm_goal.x + x_dist
            self.cf_goals[swarm_id].y = swarm_goal.y + y_dist
            self.cf_goals[swarm_id].z = swarm_goal.z + z_dist
            self.cf_goals[swarm_id].yaw = yaw

        # Update crazyflies based on swarm ID
        for _, cf_attrs in crazyflies.items():
            if rospy.is_shutdown():
                break
            cf_id = cf_attrs["swarm_id"]
            try:
                cf_attrs["goal"].x = self.cf_goals[cf_id].x
                cf_attrs["goal"].y = self.cf_goals[cf_id].y
                cf_attrs["goal"].z = self.cf_goals[cf_id].z
                cf_attrs["goal"].yaw = self.cf_goals[cf_id].yaw
            except KeyError:
                # Pass, keys arn't initialized yet because of new formation
                pass

    def compute_cf_goals_vel(self, crazyflies, swarm_goal_vel):
        """Compute goal of each crazyflie based on target velocity of the swarm

        Args:
            crazyflies (dict): Information of each Crazyflie
            swarm_goal_vel (Twist): Goal of the swarm
        """
        # Useless?
        for _, cf_attrs in crazyflies.items():
            cf_attrs["goal"].x += swarm_goal_vel.linear.x
            cf_attrs["goal"].y += swarm_goal_vel.linear.y
            cf_attrs["goal"].z += swarm_goal_vel.linear.z
            cf_attrs["goal"].yaw += swarm_goal_vel.angular.z

    def land_extra_cf(self):
        """Land CFs in extra.

        """
        x_land = 0

        for _ in range(self.n_cf, self.n_cf + self.n_cf_landed + 1):
            land_goal = Position()
            land_goal.x = x_land
            x_land += 0.25

            self.cf_goals[id] = land_goal

    # Methods depending on formation
    def set_n_cf(self, n_cf):
        """Make sure there number of CF is supported by formation and sets it

        Args:
            n (int): Number of CF
        """
        if n_cf > 0:
            self.n_cf = n_cf
            rospy.loginfo("Formation made of %i crazyflies" % self.n_cf)
        else:
            self.n_cf = 0
            rospy.logerr("Unsuported number of CFs")

    def compute_start_positions(self):
        """Compute start position of each CF
        """
        pass

    def compute_swarm_pose(self, crazyflie_list):
        """Compute pose of the swarm

        Args:
            crazyflie_list (dict of dict): Attrs of each CF

        Returns:
            Pose: Swarm Pose
        """

        # To simplify, swarm pose is the average of all the poses
        swarm_pose = Pose()

        x_vals = []
        y_vals = []
        z_vals = []
        yaw = []

        for _, cf_attrs in crazyflie_list.items():
            if cf_attrs["swarm_id"] > self.n_cf - 1:
                # Don't count landed CFs
                break

            pose = cf_attrs["pose"].pose
            x_vals.append(pose.position.x)
            y_vals.append(pose.position.y)
            z_vals.append(pose.position.z)
            yaw.append(yaw_from_quat(pose.orientation))

        swarm_pose.position.x = np.mean(x_vals)
        swarm_pose.position.y = np.mean(y_vals)
        swarm_pose.position.z = np.mean(z_vals)
        swarm_pose.orientation = quat_from_yaw(np.mean(yaw))

        return swarm_pose

    def update_scale(self):
        """Compute new positions of CFs based on new scale

        Unique to each formation

        """
        pass


def calculate_rot(start_orientation, rot):
    """Apply rotation to quaternion

    Args:
        start_orientation (Quaternion): Initial orientation
        rot (Vector3): Angular speed to apply

    Returns:
        Quaternion: Result
    """
    rot_q = quaternion_from_euler(rot.x, rot.y, rot.z)
    orig_q = [start_orientation.x,
              start_orientation.y,
              start_orientation.z,
              start_orientation.w]
    res_q = quaternion_multiply(rot_q, orig_q)

    res_msg = Quaternion()
    res_msg.x = res_q[0]
    res_msg.y = res_q[1]
    res_msg.z = res_q[2]
    res_msg.w = res_q[3]

    return res_msg

def yaw_from_quat(quaternion):
    """Returns yaw from a quaternion

    Args:
        quaternion (Quaternion)

    Returns:
        float: Yaw
    """
    _, _, yaw = euler_from_quaternion([quaternion.x,
                                       quaternion.y,
                                       quaternion.z,
                                       quaternion.w])
    return yaw

def quat_from_yaw(yaw):
    """Compute a quaternion from yaw

    Pitch and roll are considered zero

    Args:
        yaw (float): Yaw

    Returns:
        Quaternion
    """
    x_quat, y_quat, z_quat, w_quat = quaternion_from_euler(0, 0, yaw)
    return Quaternion(x_quat, y_quat, z_quat, w_quat)
