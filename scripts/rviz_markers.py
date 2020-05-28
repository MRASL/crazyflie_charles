#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose



class RvizMarkers:
    def __init__(self):
        self.bedMsg = get_bed_msg()
        self.goalMsg = get_goal_msg()

        self.rate = rospy.Rate(100)

        rospy.Subscriber("goal_swarm", Pose, self.update_goal)

        self.markerPub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

    def update_goal(self, goal):
        self.goalMsg.pose.position.x = goal.position.x
        self.goalMsg.pose.position.y = goal.position.y
        self.goalMsg.pose.position.z = goal.position.z

    def publish(self):
        while not rospy.is_shutdown():
            self.markerPub.publish(self.bedMsg)
            self.rate.sleep()

            self.markerPub.publish(self.goalMsg)
            self.rate.sleep()


def get_bed_msg():
    bedMsg = Marker()

    bedMsg.header.frame_id = "world"
    bedMsg.header.stamp = rospy.Time()

    bedMsg.ns = "bed"
    bedMsg.id = 0
    bedMsg.type = 1      # Cube
    bedMsg.action = 0    # Add

    bedMsg.pose.position.x = 1
    bedMsg.pose.position.y = 1.4
    bedMsg.pose.position.z = 0.1

    bedMsg.pose.orientation.x = 0.0
    bedMsg.pose.orientation.y = 0.0
    bedMsg.pose.orientation.z = 0.0
    bedMsg.pose.orientation.w = 1.0

    bedMsg.scale.x = 2
    bedMsg.scale.y = 1.50
    bedMsg.scale.z = 0.2

    bedMsg.color.a = 0.75
    bedMsg.color.r = 0.55
    bedMsg.color.g = 0.56
    bedMsg.color.b = 0.57

    return bedMsg


def get_goal_msg():
    msg = Marker()

    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time()

    msg.ns = "goal"
    msg.id = 0
    msg.type = 2      # Cube
    msg.action = 0    # Add

    msg.pose.position.x = 1
    msg.pose.position.y = 1
    msg.pose.position.z = 1

    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    msg.scale.x = 0.03
    msg.scale.y = 0.03
    msg.scale.z = 0.03

    msg.color.a = 0.8
    msg.color.r = 1
    msg.color.g = 0
    msg.color.b = 0
    
    return msg


if __name__ == '__main__':
    rospy.init_node('rviz_markers', anonymous=False)

    rviz_markers = RvizMarkers()

    rviz_markers.publish()



