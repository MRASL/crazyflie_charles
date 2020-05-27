#!/usr/bin/env python

"""
Script to map inputs of the controller to services and teleop CF in manual mode
"""

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

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


class Controller():
    def __init__(self, joy_topic):
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

        # Subscribe to the joystick at the end to make sure that all required services were found
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        
        # Parameters
        self._buttons = None   # Store last buttons
        self._to_teleop = False

    def _joyChanged(self, data):
        # Read the buttons
        self.getButtons(data)
        self.getAxis()

    def getAxis(self):
        pass

    def getButtons(self, data):
        """
        Circle: Emergency
        Triangle:  
        Square: TakeOff
        Cross: Land

        """
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]: # If button changed
                if i == CIRCLE and data.buttons[i] == 1:
                    self._emergency()
                if i == L2 and data.buttons[i] == 1:
                    self._toggleTeleop()

                if not self.in_teleop():
                    if i == CROSS and data.buttons[i] == 1: 
                        self._land()
                    if i == SQUARE and data.buttons[i] == 1:
                        self._takeoff()

                    if i == R2 and data.buttons[i] == 1:
                        self._stop()

                # if i == self._L2 and data.buttons[i] == 1:
                #     value = int(rospy.get_param("ring/headlightEnable"))
                #     if value == 0:
                #         rospy.set_param("ring/headlightEnable", 1)
                #     else:
                #         rospy.set_param("ring/headlightEnable", 0)
                #     self._update_params(["ring/headlightEnable"])
                #     rospy.loginfo('Head light: %s'  % (not value))

        self._buttons = data.buttons

    def _toggleTeleop(self):
        self._to_teleop = not self._to_teleop
        self._toggleTeleopServ()  # Toggle in swarm controller
        print("Teleop set to : %s" % self._to_teleop)

    def in_teleop(self):
        return self._to_teleop

    def execute(self):
        while not rospy.is_shutdown():
            # Publish on the topics
            pass

if __name__ == '__main__':
    rospy.init_node('crazyflie_joy_controller', anonymous=True)
    
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(joy_topic)

    rospy.spin()
