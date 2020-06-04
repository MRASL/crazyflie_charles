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
LS = 10
RS = 11

# - mean buttons are inversed
PAD_L_R = -9
PAD_U_D = 10

#! Only cf1 name is supported for teleop TODO #16

class Axis:
    """Represents an axis

    Attributes:
        axis_num (int): Index of the axis to read
        max_vel (int): Maximum value for velocity control
        max_goal (int): Maximum value for goal control 
    """
    def __init__(self):

        self.axis_num = 0
        self.max_vel = 0
        self.max_goal = 0

class Axes:
    """All the axis of a CF

    Attributes:
        x (Axis): X axis
        y (Axis): Y axis
        z (Axis): Z axis
        yaw (Axis): Yaw axis
    """
    def __init__(self):
        self.x = Axis()
        self.y = Axis()
        self.z = Axis()
        self.yaw = Axis()

class Controller():
    """Interface with the joystick
    """
    def __init__(self, joy_topic, to_sim):
        """Init

        Args:
            joy_topic (str): Topic with joystick values
            to_sim (bool): True if simulation is activated
        """
        # Attributes
        
        self._to_sim = to_sim #: bool
        self._buttons = None  #: list: previous state of the buttons
        self._to_teleop = False #: bool: in automatic or teleop
        self.rate = rospy.Rate(100) #: rospy.Rate: Publishing rate 
        
        self._buttonsAxes = None #: list: previous sate of the buttons on the axes

        # Init services
        self._init_services()
        
        # Subscriber
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        
        # Publisher
        # TODO Update to publish on cf_names from params
        if not to_sim:
            self.vel_publisher = rospy.Publisher("cf1/cmd_vel", Twist, queue_size=1)
            self.vel_msg = Twist()
        
        self.goal_vel_publisher = rospy.Publisher("swarm_goal_vel", Twist, queue_size=1)
        self.goal_vel_msg = Twist()

        # Axis parameters
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
        self.axes.yaw.max_goal = rospy.get_param("~yaw_goal_max", 0.05)

    def _init_services(self):
        """Init services
        """
        if not self._to_sim:
            rospy.loginfo("Joy: waiting for params service")
            rospy.wait_for_service('update_params')
            rospy.loginfo("Joy: found update_params service")
            self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

            rospy.loginfo("Joy: waiting for emergency service")
            rospy.wait_for_service('emergency')
            rospy.loginfo("Joy: found emergency service")
            self._emergency = rospy.ServiceProxy('emergency', Empty)

        rospy.loginfo("Joy: waiting for services")

        # rospy.loginfo("Joy: waiting for toggle_teleop service")
        rospy.wait_for_service('/toggle_teleop')
        # rospy.loginfo("Joy: found toggleTeleop service")
        self._toggleTeleopServ = rospy.ServiceProxy('/toggleTeleop', Empty)
        
        # rospy.loginfo("Joy: waiting for land service")
        rospy.wait_for_service('land')
        # rospy.loginfo("Joy: found land service")
        self._land = rospy.ServiceProxy('land', Empty)

        # rospy.loginfo("Joy: waiting for take_off service")
        rospy.wait_for_service('take_off')
        # rospy.loginfo("Joy: found take_off service")
        self._takeoff = rospy.ServiceProxy('take_off', Empty)

        # rospy.loginfo("Joy: waiting for stop service")
        rospy.wait_for_service('stop')
        # rospy.loginfo("Joy: found stop service")
        self._stop = rospy.ServiceProxy('stop', Empty)

        rospy.wait_for_service('formation_inc_scale')
        self._formation_inc_scale = rospy.ServiceProxy('formation_inc_scale', Empty)

        rospy.wait_for_service('formation_dec_scale')
        self._formation_dec_scale = rospy.ServiceProxy('formation_dec_scale', Empty)

        rospy.wait_for_service('toggle_ctrl_mode')
        self._toggle_abs_ctrl_mode = rospy.ServiceProxy('toggle_ctrl_mode', Empty)

        rospy.loginfo("Joy: found services")

    def _joyChanged(self, data):
        """Called when data is received from the joystick

        Args:
            data (Joy): Joystick data
        """
        # Read the buttons
        self._getButtons(data.buttons)
        self._getButtonsAxes(data.axes)
        
        if self.in_teleop():
            self.vel_msg.linear.x = self._getAxis(data.axes, self.axes.x)
            self.vel_msg.linear.y = self._getAxis(data.axes, self.axes.y)
            self.vel_msg.linear.z = self._getAxis(data.axes, self.axes.z)
            self.vel_msg.angular.z = self._getAxis(data.axes, self.axes.yaw)
        
        else:
            self.goal_vel_msg.linear.x = self._getAxis(data.axes, self.axes.x, False)
            self.goal_vel_msg.linear.y = self._getAxis(data.axes, self.axes.y, False)
            self.goal_vel_msg.linear.z = self._getAxis(data.axes, self.axes.z, False)
            self.goal_vel_msg.angular.z = self._getAxis(data.axes, self.axes.yaw, False)

    def _getAxis(self, axesData, axisToRead, measureVel=True):
        """Find the value of the axis

        Args:
            axesData (list of int): Value of all the joystick axes
            axisToRead (Axis): Axis to read
            measureVel (bool, optional): To measure velocity or goal. Defaults to True.

        Returns:
            int: Read value. Input from -1 to 1, output from -max to max
        """
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
        """Find pressed buttons

        Args:
            buttonsData (list of int): buttons values
        
        Notes:
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
                        self._takeOffSwarm()

                    if i == R2 and buttonsData[i] == 1:
                        self._stop()

                    if i == TRIANGLE and buttonsData[i] == 1:
                        self._toggle_abs_ctrl_mode()



                # if i == self._L2 and buttonsData[i] == 1:
                #     value = int(rospy.get_param("ring/headlightEnable"))
                #     if value == 0:
                #         rospy.set_param("ring/headlightEnable", 1)
                #     else:
                #         rospy.set_param("ring/headlightEnable", 0)
                #     self._update_params(["ring/headlightEnable"])
                #     rospy.loginfo('Head light: %s'  % (not value))

        self._buttons = buttonsData

    def _getButtonsAxes(self, axesData):
        for i in range(0, len(axesData)):
            if self._buttonsAxes == None or axesData[i] != self._buttonsAxes[i]: # If button changed
                if not self.in_teleop():
                    if i == abs(PAD_U_D):
                        val = axesData[i]
                        if PAD_U_D < 0: val = val*-1

                        if val == -1: self._formation_dec_scale()
                        elif val == 1: self._formation_inc_scale()

                    if i == abs(PAD_L_R):
                        # Change formation
                        pass

        self._buttonsAxes = axesData

    def _toggleTeleop(self):
        """Toggle between teleop and automatic mode

        In teleop, CF is piloted with joystick.
        In automatic, CF goal is controlled with joystick
        """
        if not self._to_sim:
            self._to_teleop = not self._to_teleop
            self._toggleTeleopServ()  # Toggle in swarm controller
            print("Teleop set to : %s" % self._to_teleop)

        else:
            rospy.logwarn("Teleop not supported in simulation")

    def _takeOffSwarm(self):
        """Take off all the CF in the swarm 
        """
        self._takeoff()

    def in_teleop(self):
        """Return teleop value

        Returns:
            bool: True if in teleop mode
        """
        return self._to_teleop

    def execute(self):
        """Loop as long as alive
        """
        while not rospy.is_shutdown():
            if self.in_teleop():
                self.vel_publisher.publish(self.vel_msg)

            else:
                self.goal_vel_publisher.publish(self.goal_vel_msg)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joy_controller', anonymous=False)
    
    joy_topic = rospy.get_param("~joy_topic", "joy")
    to_sim = rospy.get_param("~to_sim", "False")
    controller = Controller(joy_topic, to_sim)

    controller.execute()
