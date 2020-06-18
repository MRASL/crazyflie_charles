#!/usr/bin/env python

"""
Class to act as the crazyflie in simulation. Publish position of CF based on cmd_x received
"""

import rospy
from tf.transformations import quaternion_from_euler

from crazyflie_driver.msg import Position, Hover
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty

STARTING_POSITIONS = {'/cf1': [0.0, 0.0, 0.0],
                      '/cf2': [0.0, 0.5, 0.0],
                      '/cf3': [0.0, 1.0, 0.0],
                      '/cf4': [0.5, 0.0, 0.0],
                      '/cf5': [0.5, 0.5, 0.0],
                      '/cf6': [0.5, 1.0, 0.0],
                      '/cf7': [1.0, 0.0, 0.0],
                      '/cf8': [1.0, 0.5, 0.0],
                      '/cf9': [1.0, 1.0, 0.0],}

class CrazyflieSim(object):
    """To simulate position of CF based on received cmd.
    """
    def __init__(self, cf_id):
        self.cf_id = '/' + cf_id

        self.world_frame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(100)

        # Declare publishers
        self.position_pub = rospy.Publisher(self.cf_id + '/pose', PoseStamped, queue_size=1)
        self.position = PoseStamped()
        self.position.header.seq = 0
        self.position.header.stamp = rospy.Time.now()
        self.position.header.frame_id = self.world_frame
        self.position.pose.orientation.w = 1

        # rospy.logwarn("WAITING")
        # rospy.sleep(30)
        # rospy.logwarn("DONE")

        if self.cf_id in STARTING_POSITIONS.keys():
            starting_pos = STARTING_POSITIONS[self.cf_id]
            self.position.pose.position.x = starting_pos[0]
            self.position.pose.position.y = starting_pos[1]
            self.position.pose.position.z = starting_pos[2]

        else:
            self.position.pose.position.x = 0
            self.position.pose.position.y = 0
            self.position.pose.position.z = 0

        # Declare subscriptions and services
        rospy.Subscriber('%s/cmd_vel' % self.cf_id, Twist, self._cmd_vel_handler)
        rospy.Subscriber('%s/cmd_hovering' % self.cf_id, Hover, self._cmd_hover_handler)
        rospy.Subscriber('%s/cmd_position' % self.cf_id, Position, self._cmd_pos_handler)
        rospy.Subscriber('%s/cmd_stop' % self.cf_id, Empty, self._cmd_pos_handler)

    def send_pose(self):
        """Publish current pose of CF
        """
        while not rospy.is_shutdown():
            self.position.header.seq += 1
            self.position.header.stamp = rospy.Time.now()

            self.position_pub.publish(self.position)

            self.rate.sleep()

    def _cmd_vel_handler(self, vel_data):
        # rospy.logwarn("cmd_vel not implemented in simulation")
        pass

    def _cmd_hover_handler(self, _):
        rospy.logwarn("cmd_hovering not implemented in simulation")

    def _cmd_pos_handler(self, pos_data):
        self.position.pose.position.x = pos_data.x
        self.position.pose.position.y = pos_data.y
        self.position.pose.position.z = pos_data.z

        [x_quat, y_quat, z_quat, w_quat] = quaternion_from_euler(0, 0, pos_data.yaw)
        self.position.pose.orientation.x = x_quat
        self.position.pose.orientation.y = y_quat
        self.position.pose.orientation.z = z_quat
        self.position.pose.orientation.w = w_quat

    def _cmd_stop_handler(self, _):
        rospy.logwarn("cmd_stop not implemented in simulation")

if __name__ == '__main__':
    # Launch node
    rospy.init_node('cf_sim', anonymous=False)
    CF_ID = rospy.get_param("~cf_name", "cf_default")

    # Get params
    CF_SIM = CrazyflieSim(CF_ID)

    while not rospy.is_shutdown():
        CF_SIM.send_pose()
    