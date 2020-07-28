#!/usr/bin/env python

"""
Script to map inputs of the controller to services and teleop CF in manual mode
"""
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from swarm_manager.srv import JoyButton

class Controller(object):
    """Interface with the joystick
    """
    def __init__(self, joy_topic, joy_type):
        """Init

        Args:
            joy_topic (str): Topic with joystick values
            teleop (bool): True if CF is to be controlled directly /w controller
        """
        # Attributes
        self._buttons = None  #: list: previous state of the buttons
        self._buttons_axes = None #: list: previous sate of the buttons on the axes

        self.rate = rospy.Rate(10) #: rospy.Rate: Publishing rate

        # Init services
        self._init_services()

        # Subscriber
        rospy.Subscriber(joy_topic, Joy, self._joy_changed)

        # Publisher
        self.goal_vel_publisher = rospy.Publisher("/joy_swarm_vel", Twist, queue_size=1)
        self.goal_vel_msg = Twist()

        # Axis parameters
        joy_params = rospy.get_param(joy_type)
        self._button_mapping = joy_params["buttons"]
        self._button_axes_mapping = joy_params["buttons_axes"]

        # Convert keys to int
        for key in self._button_mapping.keys():
            self._button_mapping[int(key)] = self._button_mapping.pop(key)

        for key in self._button_axes_mapping.keys():
            self._button_axes_mapping[int(key)] = self._button_axes_mapping.pop(key)

        self.axes = Axes()

        self.axes.x_axis.axis_num = joy_params["axes"]["x"]
        self.axes.y_axi.axis_num = joy_params["axes"]["y"]
        self.axes.z_axis.axis_num = joy_params["axes"]["z"]
        self.axes.yaw_axis.axis_num = joy_params["axes"]["yaw"]

        self.axes.x_axis.max_vel = joy_params["max_vel"]["x"]
        self.axes.y_axi.max_vel = joy_params["max_vel"]["y"]
        self.axes.z_axis.max_vel = joy_params["max_vel"]["z"]
        self.axes.yaw_axis.max_vel = joy_params["max_vel"]["yaw"]

        self.axes.x_axis.max_goal = joy_params["max_goal"]["x"]
        self.axes.y_axi.max_goal = joy_params["max_goal"]["y"]
        self.axes.z_axis.max_goal = joy_params["max_goal"]["z"]
        self.axes.yaw_axis.max_goal = joy_params["max_goal"]["yaw"]

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

        rospy.wait_for_service('/joy_button')
        self._button_pressed = rospy.ServiceProxy('/joy_button', JoyButton)

        rospy.loginfo("Joy: found services")

    def _joy_changed(self, data):
        """Called when data is received from the joystick

        Args:
            data (Joy): Joystick data
        """
        # Read the buttons
        self._get_buttons(data.buttons)
        self._get_buttons_axes(data.axes)

        # if self.in_teleop():
        #     self.cf_vel_msg.linear.x = get_axis(data.axes, self.axes.x_axis)
        #     self.cf_vel_msg.linear.y = get_axis(data.axes, self.axes.y_axi)
        #     self.cf_vel_msg.linear.z = get_axis(data.axes, self.axes.z_axis)
        #     self.cf_vel_msg.angular.z = get_axis(data.axes, self.axes.yaw_axis)

        self.goal_vel_msg.linear.x = get_axis(data.axes, self.axes.x_axis, False)
        self.goal_vel_msg.linear.y = get_axis(data.axes, self.axes.y_axi, False)
        self.goal_vel_msg.linear.z = get_axis(data.axes, self.axes.z_axis, False)
        self.goal_vel_msg.angular.z = get_axis(data.axes, self.axes.yaw_axis, False)

        rospy.sleep(0.01)

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
        for idx, cur_val in enumerate(buttons_data):
            if self._buttons is None or cur_val != self._buttons[idx]: # If button changed
                if cur_val == 1:
                    self._button_pressed(button=self._button_mapping[idx])

        self._buttons = buttons_data

    def _get_buttons_axes(self, axes_data):
        """Find pressed buttons that are on an axis

        Args:
            axes_data (list): All axes values
        """
        for idx, cur_val in enumerate(axes_data):
            # Check axis is a button
            if idx in self._button_axes_mapping.keys():
                # If button changed
                if self._buttons_axes is None or cur_val != self._buttons_axes[idx]:

                    # If button is pressed
                    if abs(cur_val) == 1:
                        self._button_pressed(button=self._button_axes_mapping[idx*cur_val])

        self._buttons_axes = axes_data

    def execute(self):
        """Loop as long as alive
        """
        while not rospy.is_shutdown():
            self.goal_vel_publisher.publish(self.goal_vel_msg)

            self.rate.sleep()

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
    JOY_TYPE = rospy.get_param("~joy_type", "ds4")

    CONTROLLER = Controller(JOY_TOPIC, JOY_TYPE)

    CONTROLLER.execute()
