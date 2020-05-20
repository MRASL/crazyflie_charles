#!/usr/bin/env python  

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def handle_crazyflie_pose(msg, cf_name, world):
    br = tf.TransformBroadcaster()
    # pos = ()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
                     rospy.Time.now(),
                     '/crazyflie/%s' % cf_name,
                     '/%s' % world)

if __name__ == '__main__':
    rospy.init_node('crazyflie_tf_broadcaster')

    world = rospy.get_param('~world', 'world')
    cf_name = rospy.get_param('~cf_name', 'test')
    
    
    rospy.Subscriber('/%s/pose' % cf_name,
                     PoseStamped,
                     handle_crazyflie_pose,
                     cf_name,
                     world)
    rospy.spin()