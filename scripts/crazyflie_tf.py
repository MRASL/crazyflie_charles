#!/usr/bin/env python  

"""
Script to create a frame from the crazyflie current position
"""

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def handle_crazyflie_pose(msg, args):
    br = tf.TransformBroadcaster()
    frame = args[0]
    world = args[1]

    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
                     rospy.Time.now(),
                     '/%s' % frame,
                     '/%s' % world)

if __name__ == '__main__':
    rospy.init_node('crazyflie_tf_broadcaster')

    world = rospy.get_param('~world', 'world')
    cf_name = rospy.get_param('~cf_name')
    frame = rospy.get_param('~frame')

    rospy.Subscriber('/%s/pose' % cf_name,
                     PoseStamped,
                     handle_crazyflie_pose,
                     (frame, world))
    rospy.spin()