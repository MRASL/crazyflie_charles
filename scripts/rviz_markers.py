#!/usr/bin/env python

"""Generate rviz markers

:: _Rviz Markers wiki:
    http://wiki.ros.org/rviz/DisplayTypes/Marker
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped


class RvizMarkers:
    def __init__(self, cf_list):
        self.bed_msg = self.init_bed_msg()
        self.goal_msg, self.goal_pose_msg = self.init_goal_msg()
        self.cf_pose_msgs = {}
        self.cf_list = cf_list
        
        self.rate = rospy.Rate(100)


        rospy.Subscriber("swarm_goal", Pose, self.update_goal)

        for each_cf in cf_list:
            self.cf_pose_msgs[each_cf] = self.init_cf_marker(each_cf)

            rospy.Subscriber("/%s/pose" % each_cf, PoseStamped, self.update_pose, each_cf)

        self.markerPub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

    def init_bed_msg(self):
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

    def init_goal_msg(self):
        msg = Marker()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time()
        msg.ns = "goal"
        msg.id = 0
        msg.type = 2      # Sphere
        msg.action = 0    # Add
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.03
        msg.scale.y = 0.03
        msg.scale.z = 0.03
        msg.color.a = 0.8
        msg.color.r = 1
        msg.color.g = 0
        msg.color.b = 0

        msg_pose = Marker()
        msg_pose.header.frame_id = "world"
        msg_pose.header.stamp = rospy.Time()
        msg_pose.ns = "goal"
        msg_pose.id = 1
        msg_pose.type = 0      # Arrow
        msg_pose.action = 0    # Add
        msg_pose.pose.position.x = 0
        msg_pose.pose.position.y = 0
        msg_pose.pose.position.z = 0
        msg_pose.pose.orientation.x = 0.0
        msg_pose.pose.orientation.y = 0.
        msg_pose.pose.orientation.z = 0.0
        msg_pose.pose.orientation.w = 1.0
        msg_pose.scale.x = 0.08
        msg_pose.scale.y = 0.01
        msg_pose.scale.z = 0.01
        msg_pose.color.a = 0.8
        msg_pose.color.r = 1
        msg_pose.color.g = 0
        msg_pose.color.b = 0
        
        return [msg, msg_pose]

    def init_cf_marker(self, cf_name):
        marker_msg = Marker()

        marker_msg.header.frame_id = "world"
        marker_msg.header.stamp = rospy.Time()
        marker_msg.ns = cf_name
        marker_msg.id = 0
        marker_msg.type = 2      # Sphere
        marker_msg.action = 0    # Add
        marker_msg.pose.position.x = 0
        marker_msg.pose.position.y = 0
        marker_msg.pose.position.z = 0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.05
        marker_msg.color.a = 0.7
        marker_msg.color.r = 0
        marker_msg.color.g = 0
        marker_msg.color.b = 1

        pose_msg = Marker()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = rospy.Time()
        pose_msg.ns = cf_name
        pose_msg.id = 1
        pose_msg.type = 0      # Arrow
        pose_msg.action = 0    # Add
        pose_msg.pose.position.x = 0
        pose_msg.pose.position.y = 0
        pose_msg.pose.position.z = 0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        pose_msg.scale.x = 0.08
        pose_msg.scale.y = 0.01
        pose_msg.scale.z = 0.01
        pose_msg.color.a = 1
        pose_msg.color.r = 1
        pose_msg.color.g = 0
        pose_msg.color.b = 0

        print "ALLO"
        return [marker_msg, pose_msg]

    def update_pose(self, pose_msg, cf_name):
        msg_sphere, msg_pose = self.cf_pose_msgs[cf_name]

        msg_sphere.pose.position = pose_msg.pose.position
        msg_sphere.pose.orientation = pose_msg.pose.orientation

        msg_pose.pose.position = pose_msg.pose.position
        msg_pose.pose.orientation = pose_msg.pose.orientation

    def update_goal(self, goal):
        self.goal_msg.pose.position = goal.position
        self.goal_pose_msg.pose = goal

    def publish(self):
        while not rospy.is_shutdown():
            self.markerPub.publish(self.bed_msg)
            self.markerPub.publish(self.goal_msg)
            self.markerPub.publish(self.goal_pose_msg)

            for _, each_cf_msg in self.cf_pose_msgs.items():
                self.markerPub.publish(each_cf_msg[0]) # Pusblish sphere
                self.markerPub.publish(each_cf_msg[1]) # Publish arrow
            
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('rviz_markers', anonymous=False)

    cf_list = rospy.get_param("~cf_list", "['cf1']")
    rviz_markers = RvizMarkers(cf_list)

    rviz_markers.publish()



