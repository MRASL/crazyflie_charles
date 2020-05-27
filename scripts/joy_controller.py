#!/usr/bin/env python

"""
Script to map inputs of the controller to services and teleop CF in manual mode
"""

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

# Button mapping of DS4
SQUARE = 0
CROSS = 1
CIRCLE = 2
TRIANGLE = 3
L1 = 4
R1 = 5
L2 = 6
R2 = 7

#TODO: Move goal
#! Only cf1 name is supported

class Axis:
    def __init__(self):
        self.axis_num = 0
        self.max_vel = 0
        self.max_goal = 0

class Axes:
    def __init__(self):
        self.x = Axis()
        self.y = Axis()
        self.z = Axis()
        self.yaw = Axis()

class Controller():
    def __init__(self, joy_topic):
        # Subscriber
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        
        # Publisher
        # TODO Update to publish on cf_names from params
        self.vel_publisher = rospy.Publisher("cf1/cmd_vel", Twist, queue_size=1)
        self.vel_msg = Twist()
        
        self.goal_publisher = rospy.Publisher("goal_swarm", Pose, queue_size=1)
        self.goal_msg = Pose()
        self.goal_msg.orientation.x = 0
        self.goal_msg.orientation.y = 0
        self.goal_msg.orientation.z = 0
        self.goal_msg.orientation.w = 0

        # Parameters
        self._buttons = None   # Store last buttons
        self._to_teleop = False
        self.rate = rospy.Rate(100)

        self.axes = Axes()
        self.axes.x.axis_num = rospy.get_param("~x_axis", 4)
        self.axes.y.axis_num = rospy.get_param("~y_axis", 3)
        self.axes.z.axis_num = rospy.get_param("~z_axis", 2)
        self.axes.yaw.axis_num = rospy.get_param("~yaw_axis", 1)
        
        self.axes.x.max_vel = rospy.get_param("~x_velocity_max", 2.0)
        self.axes.y.max_vel = rospy.get_param("~y_velocity_max", 2.0)
        self.axes.z.max_vel = rospy.get_param("~z_velocity_max", 2.0)
        self.axes.yaw.max_vel = rospy.get_param("~yaw_velocity_max", 2.0)
        
        self.axes.x.max_goal = rospy.get_param("~x_goal_max", 0.05)
        self.axes.y.max_goal = rospy.get_param("~y_goal_max", 0.05)
        self.axes.z.max_goal = rospy.get_param("~z_goal_max", 0.05)

    def _init_services(self):
        # Subscribe to services
        rospy.loginfo("waiting for params service")
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        rospy.loginfo("waiting for toggleTeleop service")
        rospy.wait_for_service('/toggleTeleop')
        rospy.loginfo("found toggleTeleop service")
        self._toggleTeleopServ = rospy.ServiceProxy('/toggleTeleop', Empty)
        
        rospy.loginfo("waiting for land service")
        rospy.wait_for_service('land')
        rospy.loginfo("found land service")
        self._land = rospy.ServiceProxy('land', Empty)

        rospy.loginfo("waiting for takeoff service")
        rospy.wait_for_service('takeoff')
        rospy.loginfo("found takeoff service")
        self._takeoff = rospy.ServiceProxy('takeoff', Empty)

        rospy.loginfo("waiting for stop service")
        rospy.wait_for_service('stop')
        rospy.loginfo("found stop service")
        self._stop = rospy.ServiceProxy('stop', Empty)

    def _joyChanged(self, data):
        # Read the buttons
        self._getButtons(data.buttons)
        
        if self.in_teleop():
            self.vel_msg.linear.x = self._getAxis(data.axes, self.axes.x)
            self.vel_msg.linear.y = self._getAxis(data.axes, self.axes.y)
            self.vel_msg.linear.z = self._getAxis(data.axes, self.axes.z)
            self.vel_msg.angular.z = self._getAxis(data.axes, self.axes.yaw)
        
        else:
            self.goal_msg.position.x = self._getAxis(data.axes, self.axes.x, False) + self.goal_msg.position.x
            self.goal_msg.position.y = self._getAxis(data.axes, self.axes.y, False) + self.goal_msg.position.y
            self.goal_msg.position.z = self._getAxis(data.axes, self.axes.z, False) + self.goal_msg.position.z

    def _getAxis(self, axesData, axisToRead, measureVel=True):
        sign = 1.0
        if axisToRead.axis_num < 0:
            sign = -1.0
            axisToRead.axis_num = -axisToRead.axis_num

        if axisToRead.axis_num > len(axesData):
            rospy.logerr("Invalid axes number")
            return 0

        val = axesData[axisToRead.axis_num]
        max_val = axisToRead.max_vel if measureVel else axisToRead.max_goal

        return sign * val * max_val

    def _getButtons(self, buttonsData):
        """
        Circle: Emergency
        Triangle:  
        Square: TakeOff
        Cross: Land

        """
        for i in range(0, len(buttonsData)):
            if self._buttons == None or buttonsData[i] != self._buttons[i]: # If button changed
                if i == CIRCLE and buttonsData[i] == 1:
                    self._emergency()
                if i == L1 and buttonsData[i] == 1:
                    self._toggleTeleop()

                if not self.in_teleop():
                    if i == CROSS and buttonsData[i] == 1: 
                        self._land()
                    if i == SQUARE and buttonsData[i] == 1:
                        self._takeoff()

                    if i == R2 and buttonsData[i] == 1:
                        self._stop()

                # if i == self._L2 and buttonsData[i] == 1:
                #     value = int(rospy.get_param("ring/headlightEnable"))
                #     if value == 0:
                #         rospy.set_param("ring/headlightEnable", 1)
                #     else:
                #         rospy.set_param("ring/headlightEnable", 0)
                #     self._update_params(["ring/headlightEnable"])
                #     rospy.loginfo('Head light: %s'  % (not value))

        self._buttons = buttonsData

    def _toggleTeleop(self):
        self._to_teleop = not self._to_teleop
        self._toggleTeleopServ()  # Toggle in swarm controller
        print("Teleop set to : %s" % self._to_teleop)

    def in_teleop(self):
        return self._to_teleop

    def execute(self):
        while not rospy.is_shutdown():
            if self.in_teleop():
                self.vel_publisher.publish(self.vel_msg)

            else:
                self.goal_publisher.publish(self.goal_msg)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('crazyflie_joy_controller', anonymous=True)
    
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(joy_topic)

    controller.execute()
