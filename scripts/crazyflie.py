#!/usr/bin/env python

"""
Class that represents a single crazyflie

voir les commandes possibles: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c
"""

import rospy
import tf
import numpy as np

from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty as Empty_msg
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse as EmptyResponse_srv
from crazyflie_driver.srv import UpdateParams
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


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
        rospy.Service(self.cf_id + '/thrust_test', Empty_srv, self.thrust_test)
        rospy.Service(self.cf_id + '/stop', Empty_srv, self.stop)
        rospy.Service(self.cf_id + '/takeoff', Empty_srv, self.takeOffHandler)        

        # Declare publishers
        self.cmdVelPublisher = rospy.Publisher(self.cf_id + '/cmd_vel', Twist, queue_size=1)

        self.cmdHoverPusblisher = rospy.Publisher(self.cf_id + "/cmd_hover", Hover, queue_size=1)
        self.cmdHovermsg = Hover()
        self.cmdHovermsg.header.seq = 0
        self.cmdHovermsg.header.stamp = rospy.Time.now()
        self.cmdHovermsg.header.frame_id = worldFrame
        self.cmdHovermsg.yawrate = 0
        self.cmdHovermsg.vx = 0
        self.cmdHovermsg.vy = 0

        self.stop_pub = rospy.Publisher(self.cf_id + "/cmd_stop", Empty_msg, queue_size=1)
        self.stop_msg = Empty_msg()

        # Declare subscriptions
        rospy.Subscriber(self.cf_id + '/pose', PoseStamped, self.poseHandler)

        # Set parameters
        self.setParam("kalman/resetEstimation", 1)

        self.thrust = 0
        self.to_hover = False

        self.poseX = 0
        self.poseY = 0
        self.poseZ = 0

        self.initialX = 0
        self.initialY = 0
        self.initialZ = 0

        self.findInitialPose()


    def poseHandler(self, data):
        self.poseX = data.pose.position.x
        self.poseY = data.pose.position.y
        self.poseZ = data.pose.position.z

        # rospy.loginfo("CF position: %.2f, %.2f, %.2f" % (self.poseX, self.poseY, self.poseZ))

    def findInitialPose(self):
        rospy.loginfo("Estimating inital pos...")
        r = rospy.Rate(100)
        initialPose = {'x': [], 'y':[], 'z':[] } 
        while len(initialPose['x']) < 200:
            initialPose['x'].append(self.poseX)
            initialPose['y'].append(self.poseY)
            initialPose['z'].append(self.poseZ)
            r.sleep()
        
        self.initialX = np.mean(initialPose['x'])
        self.initialY = np.mean(initialPose['y'])
        self.initialZ = np.mean(initialPose['z'])
        rospy.loginfo("Initial position found at: %.2f, %.2f, %.2f" % (self.initialX, self.initialY, self.initialZ))



    
    def setParam(self, name, value):
        rospy.set_param(self.cf_id + "/" + name, value)
        self.update_params([name])

    def stop(self, req):
        self.thrust = 0
        self.to_hover = False
        return EmptyResponse_srv()

    def takeOffHandler(self, req):
        self.to_hover = True
        return EmptyResponse_srv()

    # Take off to z distance
    def takeOff(self, zDistance):
        time_range = 1 + int(10*zDistance/0.4)
        while not rospy.is_shutdown():
            rospy.loginfo("Taking off, duration %i" % time_range)
            for y in range(time_range):
                if not self.to_hover: break
                z = y / 25.0
                self.cmdHovermsg.header.seq += 1
                self.cmdHovermsg.header.stamp = rospy.Time.now()
                self.cmdHover(z)
                self.rate.sleep()

            rospy.loginfo("Hovering")
            for y in range(50):
                if not self.to_hover: break
                
                self.cmdHovermsg.header.seq += 1
                self.cmdHovermsg.header.stamp = rospy.Time.now()
                self.cmdHover(zDistance)
                self.rate.sleep()
            
            rospy.loginfo("Done")
            break

    # land from last zDistance
    def land (self):
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

    def thrust_test(self, req):
        r = rospy.Rate(100)
        rospy.loginfo("Starting thrust test")
        self.cmd_vel.linear.z = 15000

        for _ in range(0, 500):
               self.p.publish(self.cmd_vel)
               r.sleep()

        self.cmd_vel.linear.z = 0
        self.p.publish(self.cmd_vel)
        self.rate.sleep()

        rospy.loginfo("Thrust test over")
        return EmptyResponse_srv()

    def cmdVel(self, roll, pitch, yawrate, thrust):
        """
        Args:
            roll (float): Roll angle. Degrees. Positive values == roll right.
            pitch (float): Pitch angle. Degrees. Positive values == pitch
                forward/down.
            yawrate (float): Yaw angular velocity. Degrees / second. Positive
                values == turn counterclockwise.
            thrust (float): Thrust magnitude. Non-meaningful units in [0, 2^16),
                where the maximum value corresponds to maximum thrust.
        """
        msg = Twist()
        msg.linear.x = pitch
        msg.linear.y = roll
        msg.angular.z = yawrate
        msg.linear.z = thrust
        self.cmdVelPublisher.publish(msg)

    def cmdHover(self, zDistance):
        self.cmdHovermsg.zDistance = zDistance
        self.cmdHoverPusblisher.publish(self.cmdHovermsg)

    def testHover(self):
        rate = rospy.Rate(100)
        self.cmdHover(0, 0, self.hover_height)
        rate.sleep

    def testThrust(self):
        rate = rospy.Rate(100)
        self.cmdVel(0, 0, 0, self.thrust)
        rate.sleep()

    def run(self):
        if not self.to_hover:
            self.testThrust()

        else:
            self.takeOff(1)
            self.to_hover = False


if __name__ == '__main__':
    cf_id = "crazyflie1"
    rospy.init_node('hover', anonymous=True)
    rospy.loginfo('Initialisation de ' + cf_id)
    cf1 = Crazyflie(cf_id)
    
    # rospy.spin()
    start_time = rospy.get_rostime()
    toggled = False
    while not rospy.is_shutdown():
        cf1.run()