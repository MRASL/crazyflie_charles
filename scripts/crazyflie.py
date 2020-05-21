#!/usr/bin/env python

"""
Class that represents a single crazyflie
"""

import rospy
import tf
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty as Empty_msg
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse as EmptyResponse_srv
from crazyflie_driver.srv import UpdateParams
from geometry_msgs.msg import Twist


class Crazyflie:
    def __init__(self, cf_id):
        self.cf_id = '/' + cf_id

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        rospy.loginfo("waiting for update_params service...")
        rospy.wait_for_service(self.cf_id + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(self.cf_id + '/update_params', UpdateParams)

        # Declare services
        rospy.Service(self.cf_id + '/thrust_test', Empty_srv, self._thrust_test)

        # Set parameters
        self._setParam("kalman/resetEstimation", 1)

        # Set control
        self.p = rospy.Publisher(self.cf_id + '/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()


        # self.pub = rospy.Publisher(cf_id + "/cmd_hover", Hover, queue_size=1)
        # self.msg = Hover()
        # self.msg.header.seq = 0
        # self.msg.header.stamp = rospy.Time.now()
        # self.msg.header.frame_id = worldFrame
        # self.msg.yawrate = 0

        self.stop_pub = rospy.Publisher(self.cf_id + "/cmd_stop", Empty_msg, queue_size=1)
        self.stop_msg = Empty_msg()

        self.twist.linear.z = 0
        for _ in range(0, 500):
               self.p.publish(self.twist)
               self.rate.sleep()
    
    def _setParam(self, name, value):
        rospy.set_param(self.cf_id + "/" + name, value)
        self.update_params([name])

    # Take off to z distance
    def _takeOff(self, zDistance):
        time_range = 1 + int(10*zDistance/0.4)
        while not rospy.is_shutdown():
            for y in range(time_range):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = y / 25.0
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            for y in range(20):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            break

    # land from last zDistance
    def _land (self):
        # get last height
        zDistance = self.msg.zDistance

        while not rospy.is_shutdown():
            while zDistance > 0:
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
                zDistance -= 0.2
        self.stop_pub.publish(self.stop_msg)

    def _thrust_test(self, req):
        r = rospy.Rate(100)
        rospy.loginfo("Starting thrust test")
        self.twist.linear.z = 15000

        for _ in range(0, 100):
               self.p.publish(self.twist)
               self.rate.sleep()

        self.twist.linear.z = 0
        self.p.publish(self.twist)
        self.rate.sleep()
        

        rospy.loginfo("Thrust test over")
        return EmptyResponse_srv()

if __name__ == '__main__':
    cf_id = "crazyflie1"
    rospy.init_node('hover', anonymous=True)
    rospy.loginfo('Initialisation de ' + cf_id)
    cf1 = Crazyflie(cf_id)
    
    rospy.spin()
