#!/usr/bin/env python

"""
Class to act as the crazyflie in simulation. Publish position of CF based on cmd_x received

ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
:ref:`cmd-position` (crazyflie_driver/Position)
    Position command

:ref:`cmd-hovering` (crazyflie_driver/Hover)
    Hovering command

:ref:`cmd-stop` (`std_msgs/Empty`_)
    Stop CF

:ref:`cmd-vel` (`geometry_msgs/Twist`_)
    Velocity of CF

Published Topics
^^^^^^^^^^^^^^^^
:ref:`cf-pose` (geometry_msgs/PoseStamped)
    Current pose of CF

Services
^^^^^^^^
 /cfx/emergency(`std_srvs/Empty`_)
    Simulation of emergency service

Services Called
^^^^^^^^^^^^^^^
None


Parameters
^^^^^^^^^^

~cf_name"(str)

/starting_positions")[cf_name](list of float)

.. _geometry_msgs/Twist: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
.. _std_msgs/Empty: http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html
.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html

``CrazyflieSim`` class
----------------------
"""

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from crazyflie_driver.msg import Position, Hover
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty as Empty_srv
from std_msgs.msg import Empty as Empty_msg

class CrazyflieSim(object):
    """To simulate position of CF based on received cmd.
    """
    def __init__(self, cf_id, starting_pos):
        self.cf_id = '/' + cf_id

        self.world_frame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        # Declare publishers
        self.position_pub = rospy.Publisher(self.cf_id + '/pose', PoseStamped, queue_size=1)
        self.position = PoseStamped()
        self.position.header.seq = 0
        self.position.header.stamp = rospy.Time.now()
        self.position.header.frame_id = self.world_frame
        self.position.pose.orientation.w = 1

        self.position.pose.position.x = starting_pos[0]
        self.position.pose.position.y = starting_pos[1]
        self.position.pose.position.z = starting_pos[2]

        # Declare subscriptions and services
        rospy.Service(self.cf_id + '/emergency', Empty_srv, self.emergency)

        rospy.Subscriber('%s/cmd_vel' % self.cf_id, Twist, self._cmd_vel_handler)
        rospy.Subscriber('%s/cmd_hovering' % self.cf_id, Hover, self._cmd_hover_handler)
        rospy.Subscriber('%s/cmd_position' % self.cf_id, Position, self._cmd_pos_handler)
        rospy.Subscriber('%s/cmd_stop' % self.cf_id, Empty_msg, self._cmd_pos_handler)

    def send_pose(self):
        """Publish current pose of CF
        """
        while not rospy.is_shutdown():
            self.position.header.seq += 1
            self.position.header.stamp = rospy.Time.now()

            self.position_pub.publish(self.position)

            self.rate.sleep()

    def emergency(self, _):
        """Sim emergency service
        """
        rospy.logerr("%s: Emergency service called" % self.cf_id)
        return {}

    def _cmd_vel_handler(self, _):
        # rospy.logwarn("cmd_vel not implemented in simulation")
        pass

    def _cmd_hover_handler(self, hover_data):
        self.position.pose.position.x += hover_data.vx
        self.position.pose.position.y += hover_data.vy
        self.position.pose.position.z = hover_data.zDistance

        _, _, current_yaw = euler_from_quaternion([self.position.pose.orientation.x,
                                                   self.position.pose.orientation.y,
                                                   self.position.pose.orientation.z,
                                                   self.position.pose.orientation.w])
        yaw = current_yaw + hover_data.yawrate
        [x_quat, y_quat, z_quat, w_quat] = quaternion_from_euler(0, 0, yaw)
        self.position.pose.orientation.x = x_quat
        self.position.pose.orientation.y = y_quat
        self.position.pose.orientation.z = z_quat
        self.position.pose.orientation.w = w_quat

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
    START_POSITION = rospy.get_param("/starting_positions")[CF_ID]

    # Get params
    CF_SIM = CrazyflieSim(CF_ID, START_POSITION)

    while not rospy.is_shutdown():
        CF_SIM.send_pose()
