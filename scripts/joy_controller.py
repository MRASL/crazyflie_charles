#!/usr/bin/env python

"""
Script to map inputs of the controller to services
"""

import rospy
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

class Controller():
    def __init__(self, use_controller, joy_topic):
        # Map buttons to ds4
        self._square = 0
        self._cross = 1
        self._circle = 2
        self._triangle = 3
        self._L1 = 4
        self._R1 = 5
        self._L2 = 6
        self._R2 = 7

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
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

        else:
            self._land = None
            self._takeoff = None
            self._stop = None

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None   # Store last buttons
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        """
        Mapping des bouttons: 0 -> square 
                              1 -> cross
                              2 -> circle
                              3 -> triangle
                              4 -> L1
                              5 -> R1
                              6 -> L2
                              7 -> R2
                              8 -> Share
                              9 -> Options
                              10 -> LS press
                              11 -> RS press
                              12 -> PS button
                              13 -> Track pad press

        Circle: Emergency
        Triangle:  
        Square: TakeOff
        Cross: Land

        """
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]: # If button changed
                if i == self._cross and data.buttons[i] == 1 and self._land != None:
                    # print("Landing")
                    self._land()
                if i == self._circle and data.buttons[i] == 1:
                    # print("Emergency")
                    self._emergency()
                if i == self._square and data.buttons[i] == 1 and self._takeoff != None:
                    # print("Take off")
                    self._takeoff()

                if i == self._R2 and data.buttons[i] == 1 and self._stop != None:
                    # print("Stop")
                    self._stop()

                if i == self._L2 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))
                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)
                    self._update_params(["ring/headlightEnable"])
                    rospy.loginfo('Head light: %s'  % (not value))

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(use_controller, joy_topic)
    rospy.spin()
