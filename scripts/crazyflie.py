#!/usr/bin/env python

"""
Class that represents a single crazyflie

voir les commandes possibles: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c
"""

import rospy
import tf
import numpy as np
from transitions import Machine

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

        # rospy.Service(self.cf_id + '/thrust_test', Empty_srv, self.thrust_test)
        # rospy.Service(self.cf_id + '/stop', Empty_srv, self.stopServ)
        # rospy.Service(self.cf_id + '/takeoff', Empty_srv, self.takeOffServ)   
        # rospy.Service(self.cf_id + '/land', Empty_srv, self.landServ)        

        # Declare publishers
        self.cmd_vel_pub = rospy.Publisher(self.cf_id + '/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

        self.cmd_hover_pub = rospy.Publisher(self.cf_id + "/cmd_hover", Hover, queue_size=1)
        self.cmd_hover_msg = Hover()
        self.cmd_hover_msg.header.seq = 0
        self.cmd_hover_msg.header.stamp = rospy.Time.now()
        self.cmd_hover_msg.header.frame_id = self.world_frame
        self.cmd_hover_msg.yawrate = 0
        self.cmd_hover_msg.vx = 0
        self.cmd_hover_msg.vy = 0

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
        self.to_hover = False
        self.to_land = False

        self.poseX = 0
        self.poseY = 0
        self.poseZ = 0

        self.initialX = 0
        self.initialY = 0
        self.initialZ = 0

        # Define state machine
        self.states = ['pos_ctl', 'take_off', 'hover', 'land']
        self.transitions = [
            {'trigger': 'take_off_trig', 'source': 'pos_ctl', 'dest':'take_off'},
            {'trigger': 'hover_trig', 'source': 'take_off', 'dest':'hover'},
            {'trigger': 'land_trig', 'source': ['hover', 'take_off'], 'dest':'land'},
            {'trigger': 'stop_trig', 'source': '*', 'dest':'pos_ctl'},
        ]

        self.sm = Machine(model=self, states=self.states, transitions=self.transitions, initial='pos_ctl')
        # machine: Manages transitions and states
        # sm: instance of the State Machine

    def pose_handler(self, data):
        """ Update crazyflie position in world
        """
        self.poseX = data.pose.position.x
        self.poseY = data.pose.position.y
        self.poseZ = data.pose.position.z

        # rospy.loginfo("CF position: %.2f, %.2f, %.2f" % (self.poseX, self.poseY, self.poseZ))

    def findInitialPose(self):
        """ Find the initial position of the crazyflie by calculating the mean during a time interval
        """
        rospy.loginfo("Estimating inital pos...")
        r = rospy.Rate(100)
        initialPose = {'x': [], 'y':[], 'z':[] } 
        while len(initialPose['x']) < 10:
            initialPose['x'].append(self.poseX)
            initialPose['y'].append(self.poseY)
            initialPose['z'].append(self.poseZ)
            r.sleep()
        
        self.initialX = np.mean(initialPose['x'])
        self.initialY = np.mean(initialPose['y'])
        self.initialZ = np.mean(initialPose['z'])
        rospy.loginfo("Initial position: %.2f, %.2f, %.2f" % (self.initialX, self.initialY, self.initialZ))

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

    # Services handler
    def stopServ(self, req):
        self.stop_trig()
        return EmptyResponse_srv()

    def takeOffServ(self, req):
        self.take_off_trig()
        return EmptyResponse_srv()

    def landServ(self, req):
        self.land_trig()
        return EmptyResponse_srv()

    # Methods depending on state
    def on_enter_take_off(self):
        self.findInitialPose()

        dZ = 0.5
        x_goal = self.initialX
        y_goal = self.initialY
        z_goal = self.initialZ + dZ

        rospy.loginfo("Going to (%.2f, %.2f, %.2f)" % (x_goal, y_goal, z_goal))

        time_range = 1*10
        z_inc = dZ/time_range
        
        
        for i in range(time_range):
            if rospy.is_shutdown(): break

            z = i*z_inc + self.initialZ

            self.cmd_hover_msg.header.seq += 1
            self.cmd_hover_msg.header.stamp = rospy.Time.now()

            self.cmd_pos(x_goal, y_goal, z)
            self.rate.sleep()
            rospy.loginfo("Goal: (%.2f, %.2f, %.2f) \tPos: (%.2f, %.2f, %.2f)" % (x_goal, y_goal, z, self.poseX, self.poseY, self.poseZ))

        rospy.loginfo("Pos reached (%.2f, %.2f, %.2f)" % (self.poseX, self.poseY, self.poseZ))
        self.hover_trig()
    
    def on_enter_hover(self):
        rospy.loginfo("HOVERING")

        # while self.to_hover and not rospy.is_shutdown():
        #     self.cmd_pos_msg.header.seq += 1
        #     self.cmd_pos_msg.header.stamp = rospy.Time.now()
        #     self.cmd_pos(x_goal, y_goal, z_goal)
        #     self.rate.sleep()

    def on_enter_land(self):
        zGround = 0.2

        x_start = self.poseX
        y_start = self.poseY
        z_start = self.poseZ

        dZ = z_start - zGround

        rospy.loginfo("Landing")

        time_range = 2*10

        z_dec = dZ/time_range
        
        for i in range(time_range):
            if rospy.is_shutdown(): break

            z = self.initialZ - i*z_dec 

            self.cmd_hover_msg.header.seq += 1
            self.cmd_hover_msg.header.stamp = rospy.Time.now()

            # self.cmd_pos(x_start, y_start, z)
            self.rate.sleep()
            rospy.loginfo("Goal: (%.2f, %.2f, %.2f) \tPos: (%.2f, %.2f, %.2f)" % (x_start, y_start, z, self.poseX, self.poseY, self.poseZ))

        rospy.loginfo("Landed (%.2f, %.2f, %.2f)" % (self.poseX, self.poseY, self.poseZ))

        self.stop_trig()

    def on_enter_pos_ctl(self):
        rospy.loginfo("Stoping")
        self.cmd_vel(0, 0, 0, 0)
        self.rate.sleep()

    # TODO
    def thrust_test(self, req):
        r = rospy.Rate(100)
        rospy.loginfo("Starting thrust test")
        self.cmd_vel_msg.linear.z = 15000

        for _ in range(0, 500):
               self.cmd_vel_pub.publish(self.cmd_vel_msg)
               r.sleep()

        self.cmd_vel_msg.linear.z = 0
        self.cmd_vel_pub(self.cmd_vel_msg)
        self.rate.sleep()

        rospy.loginfo("Thrust test over")
        return EmptyResponse_srv()

    def testThrust(self):
        rate = rospy.Rate(100)
        self.cmd_vel(0, 0, 0, self.thrust)
        rate.sleep()

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

    def cmd_hover(self, zDistance):
        self.cmd_hover_msg.zDistance = zDistance
        self.cmd_hover_pub.publish(self.cmd_hover_msg)

    def cmd_pos(self, x, y, z):
        self.cmd_pos_msg.x = x
        self.cmd_pos_msg.y = y
        self.cmd_pos_msg.z = z
        self.cmd_pos_pub.publish(self.cmd_pos_msg)

    # Main method
    def run(self):
        rospy.loginfo(self.state)

        # if not self.to_hover and not self.to_land:
        #     self.testThrust()

        # elif self.to_hover:
        #     # self.takeOff(0.6)
        #     self.posTest()
        #     self.to_hover = False

        # elif self.to_land:
        #     self.land()
        #     self.to_land = False


if __name__ == '__main__':
    cf_id = "crazyflie1"
    rospy.init_node('hover', anonymous=True)
    rospy.loginfo('Initialisation de ' + cf_id)
    cf1 = Crazyflie(cf_id)
    
    rospy.spin()

    # start_time = rospy.get_rostime()
    # toggled = False
    # while not rospy.is_shutdown():
        # cf1.run()
