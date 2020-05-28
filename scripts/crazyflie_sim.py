#!/usr/bin/env python

"""
Class to act as the crazyflie in simulation. Publish position of CF based on cmd_x received
"""

import rospy
import tf
import numpy as np

from crazyflie_driver.msg import Position, Hover
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Empty

class CrazyflieSim:
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


        # Declare subscriptions
        rospy.Subscriber('%s/cmd_vel' % self.cf_id, Twist, self._cmd_vel_handler)
        rospy.Subscriber('%s/cmd_hovering' % self.cf_id, Hover, self._cmd_hover_handler)
        rospy.Subscriber('%s/cmd_position' % self.cf_id, Position, self._cmd_pos_handler)
        rospy.Subscriber('%s/cmd_stop' % self.cf_id, Empty, self._cmd_pos_handler)

    def send_pose(self):
        while not rospy.is_shutdown():
            self.position.header.seq += 1
            self.position.header.stamp = rospy.Time.now()
            
            self.position_pub.publish(self.position)

            self.rate.sleep()

    def _cmd_vel_handler(self, vel_data):
        # rospy.logwarn("cmd_vel not implemented in simulation")
        pass

    def _cmd_hover_handler(self, hover_data):
        rospy.logwarn("cmd_hovering not implemented in simulation")
    
    def _cmd_pos_handler(self, pos_data):
        self.position.pose.position.x = pos_data.x
        self.position.pose.position.y = pos_data.y
        self.position.pose.position.z = pos_data.z

    def _cmd_stop_handler(self, stop_data):
        rospy.logwarn("cmd_stop not implemented in simulation")

if __name__ == '__main__':
    # Launch node

    rospy.init_node('%s_sim' , anonymous=False)

    # Get params
    cf_id = rospy.get_param("~cf_name", "crazyflie")
    cf_sim = CrazyflieSim(cf_id)

    while not rospy.is_shutdown():
        cf_sim.send_pose()
    