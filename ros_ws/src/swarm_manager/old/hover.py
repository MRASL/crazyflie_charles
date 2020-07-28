#!/usr/bin/env python

"""
Script to test hovering a single CF
"""

from threading import Thread
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import Hover, Position

GND_HEIGHT = 0.2

class Crazyflie(object):
    def __init__(self, prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        rospy.loginfo("Waiting for update_params service...")
        rospy.wait_for_service('/cf1/update_params')
        rospy.loginfo("found update_params service")

        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)
        self.set_param("commander/enHighLevel", 1)
        self.set_param("stabilizer/estimator", 2) # Use EKF
        self.set_param("stabilizer/controller", 1) # 1: High lvl, 2: Mellinger
        self.set_param("kalman/resetEstimation", 1)

        self.hover_pub = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.hover_msg = Hover()
        self.hover_msg.header.seq = 0
        self.hover_msg.header.stamp = rospy.Time.now()
        self.hover_msg.header.frame_id = worldFrame
        self.hover_msg.yawrate = 0

        self.pose_pub = rospy.Publisher(prefix + "/cmd_position", Position, queue_size=1)
        self.pose_msg = Position()
        self.pose_msg.header.seq = 1
        self.pose_msg.header.stamp = rospy.Time.now()
        self.pose_msg.header.frame_id = worldFrame
        self.pose_msg.yaw = 0.0

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

        rospy.Subscriber("/%s/pose" % prefix, PoseStamped, self.pose_handler)
        self.cf_pose = PoseStamped()

    def set_param(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    def pose_handler(self, req):
        self.cf_pose = req

    # take off to z distance
    def take_off(self, zDistance):
        print "TAKE OFF"
        time_range = 1 + int(10*zDistance/0.4)
        while not rospy.is_shutdown():
            for y in range(time_range):
                self.hover_msg.vx = 0.0
                self.hover_msg.vy = 0.0
                self.hover_msg.yawrate = 0.0
                self.hover_msg.zDistance = (y / 25.0) + GND_HEIGHT
                self.hover_msg.header.seq += 1
                self.hover_msg.header.stamp = rospy.Time.now()
                self.hover_pub.publish(self.hover_msg)
                self.rate.sleep()
            for y in range(20):
                self.hover_msg.vx = 0.0
                self.hover_msg.vy = 0.0
                self.hover_msg.yawrate = 0.0
                self.hover_msg.zDistance = zDistance + GND_HEIGHT
                self.hover_msg.header.seq += 1
                self.hover_msg.header.stamp = rospy.Time.now()
                self.hover_pub.publish(self.hover_msg)
                self.rate.sleep()
            break

    # land from last zDistance
    def land (self):
        # get last height
        zDistance = self.hover_msg.zDistance - GND_HEIGHT

        while not rospy.is_shutdown():
            while zDistance > GND_HEIGHT:
                self.hover_msg.vx = 0.0
                self.hover_msg.vy = 0.0
                self.hover_msg.yawrate = 0.0
                self.hover_msg.zDistance = zDistance
                self.hover_msg.header.seq += 1
                self.hover_msg.header.stamp = rospy.Time.now()
                self.hover_pub.publish(self.hover_msg)
                self.rate.sleep()
                zDistance -= 0.05
        self.stop_pub.publish(self.stop_msg)

    def hover_pos(self, duration):
        start_position = self.cf_pose
        start_x = start_position.pose.position.x
        start_y = start_position.pose.position.y
        start_z = start_position.pose.position.z

        start = rospy.get_time()
        while not rospy.is_shutdown():
            self.pose_msg.x = start_x
            self.pose_msg.y = start_y
            self.pose_msg.z = start_z

            now = rospy.get_time()
            if now - start > duration:
                break

            self.pose_msg.header.seq += 1
            self.pose_msg.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.pose_msg)
            self.rate.sleep()

    def hover_hov(self, duration):
        start_position = self.cf_pose
        # start_x = start_position.pose.position.x
        # start_y = start_position.pose.position.y
        start_z = start_position.pose.position.z

        start = rospy.get_time()
        while not rospy.is_shutdown():
            self.hover_msg.vx = 0.0
            self.hover_msg.vy = 0.0
            self.hover_msg.yawrate = 0.0
            self.hover_msg.zDistance = start_z

            now = rospy.get_time()
            if now - start > duration:
                break

            self.hover_msg.header.seq += 1
            self.hover_msg.header.stamp = rospy.Time.now()
            self.hover_pub.publish(self.hover_msg)
            self.rate.sleep()

def handler(cf):
    print "TAKE OFF IN 3 SEC"
    rospy.sleep(3)
    cf.take_off(1.0)

    print "HOVERING"
    # cf.hover_hov(10.0)
    cf.hover_pos(10.0)

    print "LANDING"
    cf.land()

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)

    cf1 = Crazyflie("cf1")

    t1 = Thread(target=handler, args=(cf1,))
    t1.start()

    rospy.spin()
