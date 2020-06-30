#!/usr/bin/env python

"""
Script to create a frame from the crazyflie current position
"""

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def handle_crazyflie_pose(msg, args):
    """Broadcast pose to tf

    Args:
        msg (PoseStamped): CF current pose
        args (list of str): [frame, world]
    """
    pose_brd = tf.TransformBroadcaster()
    frame = args[0]
    world = args[1]

    pose_brd.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                           (msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w),
                           rospy.Time.now(),
                           '/%s' % frame,
                           '/%s' % world)


if __name__ == '__main__':
    rospy.init_node('crazyflie_tf_broadcaster')

    WORLD = rospy.get_param('~world', 'world')
    CF_NAME = rospy.get_param('~cf_name')
    FRAME = rospy.get_param('~frame')

    rospy.Subscriber('/%s/pose' % CF_NAME,
                     PoseStamped,
                     handle_crazyflie_pose,
                     (FRAME, WORLD))
    rospy.spin()
