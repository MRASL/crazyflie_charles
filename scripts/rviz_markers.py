#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

if __name__ == '__main__':
    rospy.init_node('rviz_markers', anonymous=True)

    markerPub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    markerMsg = Marker()

    markerMsg.header.frame_id = "world"
    markerMsg.header.stamp = rospy.Time()

    markerMsg.ns = "bed"
    markerMsg.id = 0
    markerMsg.type = 1      # Cube
    markerMsg.action = 0    # Add

    markerMsg.pose.position.x = 1
    markerMsg.pose.position.y = 1.4
    markerMsg.pose.position.z = 0.1

    markerMsg.pose.orientation.x = 0.0
    markerMsg.pose.orientation.y = 0.0
    markerMsg.pose.orientation.z = 0.0
    markerMsg.pose.orientation.w = 1.0

    markerMsg.scale.x = 2
    markerMsg.scale.y = 1.50
    markerMsg.scale.z = 0.2

    markerMsg.color.a = 0.75
    markerMsg.color.r = 0.55
    markerMsg.color.g = 0.56
    markerMsg.color.b = 0.57

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        markerPub.publish(markerMsg)
        r.sleep()
