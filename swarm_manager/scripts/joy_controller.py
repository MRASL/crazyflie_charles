#!/usr/bin/env python

"""
Script to map inputs of the controller to services and teleop CF in manual mode

Services:
    - None

Subscribed services:
    - From swarm_manager
        - update_params
        - emergency
        - toggle_teleop
        - land
        - take_off
        - stop
        - formation_inc_scale
        - formation_dec_scale
        - toggle_ctrl_mode
        - next_swarm_formation
        - prev_swarm_formation

Subscription:
    - from joy
        - joy_topic

Publisher:
    - /cf1/cmd_vel: Velocity of a single CF (only when in teleop)
    - /joy_swarm_vel: Velocity of the swarm

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
LS = 10
RS = 11

# - mean buttons are inversed
PAD_L_R = -9
PAD_U_D = 10

#! Only cf1 name is supported for teleop TODO #16

class Axis(object):
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

class Axes(object):
    """All the axis of a CF

    Attributes:
        x (Axis): X axis
        y (Axis): Y axis
        z (Axis): Z axis
        yaw (Axis): Yaw axis
    """
    def __init__(self):
        self.x_axis = Axis()
        self.y_axi = Axis()
        self.z_axis = Axis()
        self.yaw_axis = Axis()

class Controller(object):
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

        self._buttons_axes = None #: list: previous sate of the buttons on the axes

        # Init services
        self._init_services()

        # Subscriber
        rospy.Subscriber(joy_topic, Joy, self._joy_changed)

        # Publisher
        # TODO Update to publish on cf_names from params
        if not to_sim:
            self.vel_publisher = rospy.Publisher("cf1/cmd_vel", Twist, queue_size=1)
            self.cf_vel_msg = Twist()

        self.goal_vel_publisher = rospy.Publisher("/joy_swarm_vel", Twist, queue_size=1)
        self.goal_vel_msg = Twist()

        # Axis parameters
        self.axes = Axes()
        self.axes.x_axis.axis_num = rospy.get_param("~x_axis", 4)
        self.axes.y_axi.axis_num = rospy.get_param("~y_axis", 3)
        self.axes.z_axis.axis_num = rospy.get_param("~z_axis", 2)
        self.axes.yaw_axis.axis_num = rospy.get_param("~yaw_axis", 1)

        self.axes.x_axis.max_vel = rospy.get_param("~x_velocity_max", 2.0)
        self.axes.y_axi.max_vel = rospy.get_param("~y_velocity_max", 2.0)
        self.axes.z_axis.max_vel = rospy.get_param("~z_velocity_max", 2.0)
        self.axes.yaw_axis.max_vel = rospy.get_param("~yaw_velocity_max", 2.0)

        self.axes.x_axis.max_goal = rospy.get_param("~x_goal_max", 0.05)
        self.axes.y_axi.max_goal = rospy.get_param("~y_goal_max", 0.05)
        self.axes.z_axis.max_goal = rospy.get_param("~z_goal_max", 0.05)
        self.axes.yaw_axis.max_goal = rospy.get_param("~yaw_goal_max", 0.05)

    def _init_services(self):
        """Init services

        Service list:
            - update_params
            - emergency
            - toggle_teleop
            - land
            - take_off
            - stop
            - formation_inc_scale
            - formation_dec_scale
            - toggle_ctrl_mode
            - next_swarm_formation
            - prev_swarm_formation
        """
        # Find services
        rospy.loginfo("Joy: waiting for services...")

        if not self._to_sim:
            rospy.wait_for_service('/update_swarm_params')
            self._update_params = rospy.ServiceProxy('/update_swarm_params', UpdateParams)

            rospy.wait_for_service('/swarm_emergency')
            self._emergency = rospy.ServiceProxy('/swarm_emergency', Empty)

        rospy.wait_for_service('/toggle_teleop')
        self._toggle_teleop_srv = rospy.ServiceProxy('/toggle_teleop', Empty)

        rospy.wait_for_service('/land_swarm')
        self._land = rospy.ServiceProxy('/land_swarm', Empty)

        rospy.wait_for_service('/take_off_swarm')
        self._takeoff = rospy.ServiceProxy('/take_off_swarm', Empty)

        rospy.wait_for_service('/stop_swarm')
        self._stop = rospy.ServiceProxy('/stop_swarm', Empty)

        rospy.wait_for_service('/inc_swarm_scale')
        self._formation_inc_scale = rospy.ServiceProxy('/inc_swarm_scale', Empty)

        rospy.wait_for_service('/dec_swarm_scale')
        self._formation_dec_scale = rospy.ServiceProxy('/dec_swarm_scale', Empty)

        rospy.wait_for_service('/toggle_ctrl_mode')
        self._toggle_abs_ctrl_mode = rospy.ServiceProxy('/toggle_ctrl_mode', Empty)

        rospy.wait_for_service('/next_swarm_formation')
        self._next_swarm_formation = rospy.ServiceProxy('/next_swarm_formation', Empty)

        rospy.wait_for_service('/prev_swarm_formation')
        self._prev_swarm_formation = rospy.ServiceProxy('/prev_swarm_formation', Empty)
        rospy.loginfo("Joy: found services")

    def _joy_changed(self, data):
        """Called when data is received from the joystick

        Args:
            data (Joy): Joystick data
        """
        # Read the buttons
        self._get_buttons(data.buttons)
        self._get_buttons_axes(data.axes)

        if self.in_teleop():
            self.cf_vel_msg.linear.x = get_axis(data.axes, self.axes.x_axis)
            self.cf_vel_msg.linear.y = get_axis(data.axes, self.axes.y_axi)
            self.cf_vel_msg.linear.z = get_axis(data.axes, self.axes.z_axis)
            self.cf_vel_msg.angular.z = get_axis(data.axes, self.axes.yaw_axis)

        else:
            self.goal_vel_msg.linear.x = get_axis(data.axes, self.axes.x_axis, False)
            self.goal_vel_msg.linear.y = get_axis(data.axes, self.axes.y_axi, False)
            self.goal_vel_msg.linear.z = get_axis(data.axes, self.axes.z_axis, False)
            self.goal_vel_msg.angular.z = get_axis(data.axes, self.axes.yaw_axis, False)

    def _get_buttons(self, buttons_data):
        """Find pressed buttons

        Args:
            buttonsData (list of int): buttons values

        Notes:
            Circle: Emergency
            Triangle: Toggle between relative and absolute control
            Square: TakeOff
            Cross: Land
        """

        for i in range(0, len(buttons_data)):
            if self._buttons is None or buttons_data[i] != self._buttons[i]: # If button changed
                if i == CIRCLE and buttons_data[i] == 1:
                    self._emergency()
                if i == L1 and buttons_data[i] == 1:
                    self._toggle_teleop()

                if not self.in_teleop():
                    if i == CROSS and buttons_data[i] == 1:
                        self._land()
                    if i == SQUARE and buttons_data[i] == 1:
                        self._take_off_swarm()

                    if i == R2 and buttons_data[i] == 1:
                        self._stop()

                    if i == TRIANGLE and buttons_data[i] == 1:
                        self._toggle_abs_ctrl_mode()

                # if i == self._L2 and buttonsData[i] == 1:
                #     value = int(rospy.get_param("ring/headlightEnable"))
                #     if value == 0:
                #         rospy.set_param("ring/headlightEnable", 1)
                #     else:
                #         rospy.set_param("ring/headlightEnable", 0)
                #     self._update_params(["ring/headlightEnable"])
                #     rospy.loginfo('Head light: %s'  % (not value))

        self._buttons = buttons_data

    def _get_buttons_axes(self, axes_data):
        for i in range(0, len(axes_data)):
            # If button changed
            if self._buttons_axes is None or axes_data[i] != self._buttons_axes[i]:
                if not self.in_teleop():
                    if i == abs(PAD_U_D):
                        val = axes_data[i]
                        if PAD_U_D < 0:
                            val = val*-1

                        if val == -1:
                            self._formation_dec_scale()
                        elif val == 1:
                            self._formation_inc_scale()

                    if i == abs(PAD_L_R):
                        # Change formation
                        val = axes_data[i]
                        if PAD_L_R < 0:
                            val = val*-1

                        if val == -1:
                            self._prev_swarm_formation()
                        elif val == 1:
                            self._next_swarm_formation()

        self._buttons_axes = axes_data

    def _toggle_teleop(self):
        """Toggle between teleop and automatic mode

        In teleop, CF is piloted with joystick.
        In automatic, CF goal is controlled with joystick
        """
        if not self._to_sim:
            self._to_teleop = not self._to_teleop
            self._toggle_teleop_srv()  # Toggle in swarm controller
            print "Teleop set to : %s" % self._to_teleop

        else:
            rospy.logwarn("Teleop not supported in simulation")

    def _take_off_swarm(self):
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
                self.vel_publisher.publish(self.cf_vel_msg)

            else:
                self.goal_vel_publisher.publish(self.goal_vel_msg)

            self.rate.sleep()

def get_axis(axes_data, axis_to_read, in_teleop=True):
    """Find the value of the axis

    When in_teleop is True, controls velocity of a single CF.
    When in_teleop is False, controls velocity of swarm goal.

    The only difference is the max value of each axis.

    Args:
        axesData (list of int): Value of all the joystick axes
        axisToRead (Axis): Axis to read
        measureVel (bool, optional): To measure velocity or goal. Defaults to True.

    Returns:
        int: Read value. Input from -1 to 1, output from -max to max
    """
    sign = 1.0
    if axis_to_read.axis_num < 0:
        sign = -1.0
        axis_to_read.axis_num = -axis_to_read.axis_num

    if axis_to_read.axis_num > len(axes_data):
        rospy.logerr("Invalid axes number")
        return 0

    val = axes_data[axis_to_read.axis_num]
    max_val = axis_to_read.max_vel if in_teleop else axis_to_read.max_goal

    return sign * val * max_val

if __name__ == '__main__':
    rospy.init_node('joy_controller', anonymous=False)

    JOY_TOPIC = rospy.get_param("~joy_topic", "joy")
    TO_SIM = rospy.get_param("~to_sim", "False")
    CONTROLLER = Controller(JOY_TOPIC, TO_SIM)

    CONTROLLER.execute()
