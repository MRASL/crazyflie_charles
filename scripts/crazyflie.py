#!/usr/bin/env python

"""
Class that represents a single crazyflie

voir les commandes possibles: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c
"""

import rospy
import tf
import numpy as np
import thread

from crazyflie_driver.msg import Hover, Position
from std_msgs.msg import Empty as Empty_msg
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse as EmptyResponse_srv
from crazyflie_driver.srv import UpdateParams
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

class Crazyflie:
    def __init__(self, cf_id):
        self.cf_id = '/' + cf_id

        self.world_frame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        # Declare services
        rospy.loginfo("waiting for update_params service...")
        rospy.wait_for_service(self.cf_id + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(self.cf_id + '/update_params', UpdateParams)     

        # Declare publishers
        self.cmd_vel_pub = rospy.Publisher(self.cf_id + '/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

        self.cmd_hovering_pub = rospy.Publisher(self.cf_id + "/cmd_hovering", Hover, queue_size=1)
        self.cmd_hovering_msg = Hover()
        self.cmd_hovering_msg.header.seq = 0
        self.cmd_hovering_msg.header.stamp = rospy.Time.now()
        self.cmd_hovering_msg.header.frame_id = self.world_frame
        self.cmd_hovering_msg.yawrate = 0
        self.cmd_hovering_msg.vx = 0
        self.cmd_hovering_msg.vy = 0

        self.cmd_pos_pub = rospy.Publisher(self.cf_id + "/cmd_position", Position, queue_size=1)
        self.cmd_pos_msg = Position()
        self.cmd_pos_msg.header.seq = 1
        self.cmd_pos_msg.header.stamp = rospy.Time.now()
        self.cmd_pos_msg.header.frame_id = self.world_frame
        self.cmd_pos_msg.yaw = 0

        self.cmd_stop_pub = rospy.Publisher(self.cf_id + "/cmd_stop", Empty_msg, queue_size=1)
        self.cmd_stop_msg = Empty_msg()

        # Declare subscriptions
        rospy.Subscriber(self.cf_id + '/pose', PoseStamped, self.pose_handler)

        # Set parameters
        self.setParam("kalman/resetEstimation", 1)

        self.thrust = 0
        self.to_land = False
        self.to_hover = False

        self.pose = [0, 0 , 0]
        self.intial_pose = [0, 0, 0]
        self.goal = [0, 0, 0]

        self.states = ["take_off", "land", "hover", "stop"]
        self.state = ""

    def pose_handler(self, data):
        """ Update crazyflie position in world
        """
        self.pose[0] = data.pose.position.x
        self.pose[1] = data.pose.position.y
        self.pose[2] = data.pose.position.z

    def findInitialPose(self):
        """ Find the initial position of the crazyflie by calculating the mean during a time interval
        """
        rospy.loginfo("Estimating inital pos...")
        r = rospy.Rate(100)
        initialPose = {'x': [], 'y':[], 'z':[] } 
        while len(initialPose['x']) < 10:
            initialPose['x'].append(self.pose[0])
            initialPose['y'].append(self.pose[1])
            initialPose['z'].append(self.pose[2])
            r.sleep()
        
        self.intial_pose[0] = np.mean(initialPose['x'])
        self.intial_pose[1] = np.mean(initialPose['y'])
        self.intial_pose[2] = np.mean(initialPose['z'])
        rospy.loginfo("Initial position: {}".format(self.intial_pose))

    def setParam(self, name, value):
        """Changes the value of the given parameter.

        Args:
            name (str): The parameter's name.
            value (Any): The parameter's value.
        """

        rospy.set_param(self.cf_id + "/" + name, value)
        self.update_params([name])

    def getId(self):
        return self.cf_id

    # To change states
    def _setState(self, newState):
        if newState in self.states:
            self.state = newState
        else:
            rospy.logerr("Invalid State: %s" % newState)

    def take_off(self):
        rospy.loginfo("Take off")
        self._setState("take_off")

    def hover(self):
        rospy.loginfo("Hover")
        self._setState("hover")

    def land(self):
        rospy.loginfo("Landing")
        self._setState("land")

    def stop(self):
        rospy.loginfo("Stoping")
        self._setState("stop")

    # Methods depending on state
    def _take_off(self):
        self.findInitialPose()

        dZ = 0.5
        self.goal[0] = self.intial_pose[0]
        self.goal[1] = self.intial_pose[1]
        self.goal[2] = self.intial_pose[2] + dZ

        rospy.loginfo("Going to {}".format(self.goal))

        time_range = 1*10
        z_inc = dZ/time_range
        
        
        for i in range(time_range):
            if rospy.is_shutdown() or self.state is not "take_off": break

            z = i*z_inc + self.intial_pose[2]

            self.cmd_hovering_msg.header.seq += 1
            self.cmd_hovering_msg.header.stamp = rospy.Time.now()

            self.cmd_pos(self.goal[0], self.goal[1], z)

            self.rate.sleep()
            rospy.loginfo("Goal: (%.2f, %.2f, %.2f) \tPos: (%.2f, %.2f, %.2f)" % (self.goal[0], self.goal[0], z, self.pose[0], self.pose[1], self.pose[2]))

        rospy.loginfo("Pos reached {}".format(self.pose))

        if self.state is "take_off":
            self.hover()

    def _hover(self):

        rospy.loginfo("Hovering")
        self.cmd_pos_msg.header.seq += 1
        self.cmd_pos_msg.header.stamp = rospy.Time.now()
        self.cmd_pos(self.goal[0], self.goal[1], self.goal[2])
        self.rate.sleep()

    def _land(self):
        zGround = 0.2

        x_start = self.pose[0]
        y_start = self.pose[1]
        z_start = self.pose[2]

        dZ = z_start - zGround


        time_range = 2*10

        z_dec = dZ/time_range
        
        for i in range(time_range):
            if rospy.is_shutdown() or self.state is not "land": break

            z = z_start - i*z_dec 

            self.cmd_hovering_msg.header.seq += 1
            self.cmd_hovering_msg.header.stamp = rospy.Time.now()

            self.cmd_pos(x_start, y_start, z)

            self.rate.sleep()
            rospy.loginfo("Goal: (%.2f, %.2f, %.2f) \tPos: (%.2f, %.2f, %.2f)" % (x_start, y_start, z, self.pose[0], self.pose[1], self.pose[2]))

        rospy.loginfo("Landed {}".format(self.pose))

        self.stop()

    def _stop(self):
        self.cmd_vel(0, 0, 0, 0)
        self.rate.sleep()

    # PUblishing methods
    def cmd_vel(self, roll, pitch, yawrate, thrust):
        """
        Publish pose in cmd_vel topic
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
        self.cmd_vel_pub.publish(msg)

    def cmd_hovering(self, zDistance):
        self.cmd_hovering_msg.zDistance = zDistance
        self.cmd_hovering_pub.publish(self.cmd_hovering_msg)

    def cmd_pos(self, x, y, z):
        self.cmd_pos_msg.x = x
        self.cmd_pos_msg.y = y
        self.cmd_pos_msg.z = z
        self.cmd_pos_pub.publish(self.cmd_pos_msg)

    def run_auto(self):
        if self.state == "take_off":
            self._take_off()
        elif self.state == "hover":
            self._hover()
        elif self.state == "land":
            self._land()
        else:
            self._stop()