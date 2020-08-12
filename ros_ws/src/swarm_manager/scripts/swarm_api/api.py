#!/usr/bin/env python

"""Python API to control the swarm.

This module maked it possible to easily send command to the swarm through a Python script.

Example
-------
::

   # Formation exemple
   swarm = SwarmAPI()

   # Link joystick buttons to commands
   swarm.start_joystick("ds4")
   swarm.link_joy_button("S", swarm.take_off)
   swarm.link_joy_button("X", swarm.land)
   swarm.link_joy_button("O", swarm.emergency)
   swarm.link_joy_button("T", swarm.toggle_ctrl_mode)

   # Start swarm
   swarm.set_mode("formation")
   swarm.set_formation("v")

   swarm.take_off()
   rospy.sleep(10)

   # Change formation
   swarm.set_formation("pyramid")

``SwarmAPI`` class
------------------
"""
from math import pi
import ast
import rospy
from std_srvs.srv import Empty, SetBool
from swarm_manager.srv import JoyButton, SetGoals, GetPositions, SetMode
from formation_manager.srv import SetFormation
from geometry_msgs.msg import Twist

from swarm_api.launch_file_api import launch_joystick

class SwarmAPI(object):
    """Python API class
    """
    # Initialization
    def __init__(self):
        rospy.init_node("SwarmAPI", anonymous=False)

        self._rate = rospy.Rate(10)

        self._services = {}
        self._init_services()

        self._joy_type = None
        self._joy_buttons = None

        self._current_mode = ""

        self._joy_vel_publisher = rospy.Publisher("/joy_swarm_vel", Twist, queue_size=1)
        self._joy_vel = Twist()

    def _init_services(self):
        # Start services
        rospy.Service('/joy_button', JoyButton, self._button_srv)

        # Subscribe to srvs
        rospy.loginfo("API: waiting for services")
        self._link_service('swarm_emergency', Empty)
        self._link_service('stop_swarm', Empty)
        self._link_service('take_off_swarm', Empty)
        self._link_service('land_swarm', Empty)

        self._link_service('set_mode', SetMode)
        self._link_service('go_to', SetGoals)
        self._link_service('get_positions', GetPositions)

        self._link_service('set_swarm_formation', SetFormation)
        self._link_service('next_swarm_formation', Empty)
        self._link_service('prev_swarm_formation', Empty)
        self._link_service('inc_swarm_scale', Empty)
        self._link_service('dec_swarm_scale', Empty)
        self._link_service('toggle_ctrl_mode', Empty)

        rospy.loginfo("API: services found")

    def _link_service(self, service_name, service_type):
        """Link service

        Args:
            service_name (str): Name of the serviec
            service_type (type): Type of the service
        """
        rospy.wait_for_service('/%s' %  service_name)
        self._services[service_name] = rospy.ServiceProxy('/%s' % service_name, service_type)

    # Joystick
    def start_joystick(self, joy_type, joy_dev='js0'):
        """Initialize joystick node. See :doc:`here </getting_started/tutorials/controller_setup>`
        for a tutorial on how to add new joystick types.

        Possible types:

            * ds4

        Args:
            joy_type (:obj:`str`): Controller type.
            joy_dev (:obj:`str`, Optional): Specify joystick port. Defaults to `js0`
        """
        self._joy_type = joy_type

        joy_dev = "/dev/input/" + joy_dev
        print joy_dev

        launch_joystick(joy_type, joy_dev)

        self._joy_buttons = {}

        # Add buttons
        for _, button in rospy.get_param(joy_type)["buttons"].items():
            self._joy_buttons[button] = [None, None, None]

        # Add buttons on a axis
        for _, button in rospy.get_param(joy_type)["buttons_axes"].items():
            self._joy_buttons[button] = [None, None, None]

        # Link service
        self._link_service('set_joy_control', SetBool)

    def set_joy_control(self, to_control):
        """To enable/disable control of formation position with joystick axes.

        Args:
            to_control (bool): If True, formation will be moved by joystick axes
        """
        self._services["set_joy_control"](data=to_control)

    def link_joy_button(self, button_name, func, args=None, kwargs=None):
        """Link a button to a function call

        Args:
            button_name (:obj:`str`): Name of button, as written in ``joy_conf.yaml``.
            func (:obj:`Callable`): Function to call
            args (optional): Function args. Defaults to None. Can be a single arg or a list of args
            kwargs (:obj:`dict`, optional): Function kwargs. Defaults to None.

        Raises:
            KeyError: Invalid button name

        Example::

            swarm.start_joystick("ds4")
            swarm.link_joy_button("S", swarm.take_off)
            swarm.link_joy_button("X", swarm.land)
        """

        if args is None:
            args = []
        elif not isinstance(args, list):
            args = [args]
        if kwargs is None:
            kwargs = {}

        if button_name not in self._joy_buttons.keys():
            raise KeyError("Invalid button name %s for controller %s"%(button_name, self._joy_type))

        else:
            self._joy_buttons[button_name] = [func, args, kwargs]

    def _button_srv(self, srv_req):
        print "Button pressed: %s" % srv_req.button

        func = self._joy_buttons[srv_req.button][0]
        args = self._joy_buttons[srv_req.button][1]
        kwargs = self._joy_buttons[srv_req.button][2]

        if func is not None:
            func(*args, **kwargs)

        return {}

    # Methods
    def take_off(self):
        """ Take off all landed CFs.

        Modify ``take_off_height`` in ``swarm_conf.yaml`` to change
        take off height

        .. note::
            Will only take off landed CFs

        """
        self._services["take_off_swarm"]()

    def stop(self):
        """Stop all CFs
        """
        self._services["stop_swarm"]()

    def emergency(self):
        """Call emergency srv of all CFs
        """
        self._services["swarm_emergency"]()

    def land(self):
        """Land all CFs at their starting position.
        """
        self._services["land_swarm"]()

    def set_mode(self, new_mode):
        """Set ``SwarmController`` control mode.

        Possible modes are:
            - Automatic: CF will plot trajectory to new goals. Send ``go_to commands``
              from python script
            - Formation: Swarm moves in formation. Formation position can be moved /w joystick.

        Not implemented:
            - Pilot: Like CF client. No formation
            - Assisted: Control change of position /w joystick. No formation.

        .. note:: Modes are not case sensitive

        Args:
            new_mode (:obj:`str`): New control mode
        """
        res = self._services["set_mode"](new_mode)

        if not res.success:
            rospy.logerr("%s is not an avaible mode" % new_mode)
        else:
            rospy.loginfo("Mode set to: %s" % new_mode.lower())
            self._current_mode = new_mode.lower()

    def set_formation(self, formation_name):
        """Set swarn formation

        Args:
            formation_name (:obj:`str`): New formation name
        """
        self._services["set_swarm_formation"](formation=formation_name)

    def next_formation(self):
        """Go to next swarm formation
        """
        self._services["next_swarm_formation"]()

    def prev_formation(self):
        """Go to prev swarm formation
        """
        self._services["prev_swarm_formation"]()

    def inc_scale(self):
        """Increase formation scale by 0.5
        """
        self._services["inc_swarm_scale"]()

    def dec_scale(self):
        """Decrease formation scale by 0.5
        """
        self._services["dec_swarm_scale"]()

    def toggle_ctrl_mode(self):
        """Toggle control mode between absolute and relative.

        In absolute: x, y, z are world axis

        In relative: x, y, z depends on swarm orientation
        """
        self._services["toggle_ctrl_mode"]()

    def go_to(self, goals):
        """Move formation and/or cf to a position using the trajectory planner.

        Dict format: ``"goal_name": [x, y, z, yaw]`` where ``"goal_name"`` is either
        ``"formation"`` or ``"cf_x"``

        X, Y, Z in meters, Yaw in rad.

        Example::
            # To move formation to [2, 2, 0.5, 1.57]
            swarm.go_to({'formation': [2, 2, 0.5, 1.57]})

            # To move cf_0 to [0, 0, 0.5] and cf_1 to [1, 1, 1]
            goals = {}
            goals["cf_0"] = [0, 0, 0.5, 0]
            goals["cf_1"] = [1, 1, 1, 0]
            swarm.go_to(goals)

        Args:
            goals (:obj:`dict`): New goals
        """
        self._services["go_to"](goals=str(goals))

    def get_positions(self, cf_list=None):
        """Get current position of crazyflies

        If ``cf_list`` is ``None``, will return position of all Cfs.

        Args:
            cf_list (:obj:`list`, optional): List of cf to read positions. Defaults to None.

        Returns:
            :obj:`dict`: Positions of CFs read. ``"{cf_id": [x, y, z, yaw], ...}``
        """
        if cf_list is None:
            cf_list = []
        positions = self._services["get_positions"](cf_list=str(cf_list)).positions

        return ast.literal_eval(positions)

    def rotate_formation(self, angle, duration):
        """Rotate formation around it's center

        .. note::
            Formation control with joystick must be False to use this function
            ``swarm.set_joy_control(False)``

        Args:
            angle (float): Angle to turn [deg]
            duration (float): Rotation duration [sec]
        """
        # Convert angle to angular speed. At each publish, turns ``speed`` in rad
        speed_rad = angle/duration*pi/180 #: [rad/sec]
        delta_angle = speed_rad/10 #: [rad par 100ms]

        self._joy_vel.angular.z = delta_angle

        start_time = rospy.Time.now()
        rotate_dur = rospy.Duration(duration)

        while rospy.Time.now() - start_time < rotate_dur and not rospy.is_shutdown():
            self._joy_vel_publisher.publish(self._joy_vel)
            self._rate.sleep()

        self._joy_vel.angular.z = 0
        self._joy_vel_publisher.publish(self._joy_vel)
        self._rate.sleep()
