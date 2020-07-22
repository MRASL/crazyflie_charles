"""Python API to control the swarm.

This class is needed since the `SwarmController` needs to constantly publish each CF goal.


TODO:
    - [x] Methodes a implementer
        - [x] Set formation
        - [x] Set formation goal
        - [x] Set CF goals
        - [x] Get pose
        - [x] Relative ctrl mode
    - [x] Controle avec joystick
        - [x] Init joystick node
        - [x] Link buttons and functions
        - [x] Link d_pad
    - [ ] Modes de controle
        - [ ] Formation (meme chose que maintenant)
        - [ ] Manuel (comme avec CF client)
        - [ ] Assisted (control le deplacement)
        - [ ] Automatic
            - Control goal
"""
import ast
import rospy
from std_srvs.srv import Empty
from swarm_manager.srv import JoyButton, SetGoals, GetPositions
from formation_manager.srv import SetFormation

from launch_file_api import launch_joystick

class SwarmAPI(object):
    """API class to control the swarm
    """
    # Initialization
    def __init__(self):
        rospy.init_node("SwarmAPI", anonymous=False)

        self._services = {}
        self._init_services()

        self.joy_type = None
        self.joy_buttons = None

    def _init_services(self):
        # Subscribe to srvs
        rospy.loginfo("API: waiting for services")
        self._link_service('swarm_emergency', Empty)
        self._link_service('stop_swarm', Empty)
        self._link_service('take_off_swarm', Empty)
        self._link_service('land_swarm', Empty)

        self._link_service('set_goals', SetGoals)
        self._link_service('get_positions', GetPositions)

        self._link_service('set_swarm_formation', SetFormation)
        self._link_service('next_swarm_formation', Empty)
        self._link_service('prev_swarm_formation', Empty)
        self._link_service('inc_swarm_scale', Empty)
        self._link_service('dec_swarm_scale', Empty)
        self._link_service('toggle_ctrl_mode', Empty)


        rospy.loginfo("API: services found")

        # Start services
        rospy.Service('/joy_button', JoyButton, self._button_srv)

    def _link_service(self, service_name, service_type):
        """Link service

        Args:
            service_name (str): Name of the serviec
            service_type (type): Type of the service
        """
        rospy.wait_for_service('/%s' %  service_name)
        self._services[service_name] = rospy.ServiceProxy('/%s' % service_name, service_type)

    # Joystick
    def start_joystick(self, joy_type="ds4"):
        """Initialize joystick node

        Possible types are:
            - ds4

        Args:
            joy_type (str, optional): Controller type. Defaults to "ds4".
        """
        self.joy_type = joy_type

        launch_joystick(joy_type)

        self.joy_buttons = {}

        # Add buttons
        for _, button in rospy.get_param(joy_type)["buttons"].items():
            self.joy_buttons[button] = None

        # Add buttons on a axis
        for _, button in rospy.get_param(joy_type)["buttons_axes"].items():
            self.joy_buttons[button] = None

    def link_joy_button(self, button_name, func, args=None, kwargs=None):
        """Link a button to a function call

        Args:
            button_name (str): Name of button
            func (Callable): Function to call
            args (optional): Function args. Defaults to None. Can be a single arg or a list of args
            kwargs (dict, optional): Function kwargs. Defaults to None.

        Raises:
            KeyError: Invalid button name
        """

        if args is None:
            args = []
        elif not isinstance(args, list):
            args = [args]
        if kwargs is None:
            kwargs = {}

        if button_name not in self.joy_buttons.keys():
            raise KeyError("Invalid button name %s for controller %s"%(button_name, self.joy_type))

        else:
            self.joy_buttons[button_name] = [func, args, kwargs]

    def _button_srv(self, srv_req):
        func = self.joy_buttons[srv_req.button][0]
        args = self.joy_buttons[srv_req.button][1]
        kwargs = self.joy_buttons[srv_req.button][2]

        if func is not None:
            func(*args, **kwargs)

        return {}

    # Methods
    def take_off(self):
        """ Take off all CFs
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
        """Land all CFs
        """
        self._services["land_swarm"]()

    def set_formation(self, formation_name):
        """Set formation

        Args:
            formation_name (str): Formation name
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

    def set_goals(self, goals):
        """Set formation and/or cf goal.

        Dict format: `"goal_name": [x, y, z, yaw]` where `"goal_name"` is either
        `"formation" or "cf_x"`

        Args:
            goals (dict):
        """
        self._services["set_goals"](goals=str(goals))

    def get_positions(self, cf_list=None):
        """Get current position of crazyflies

        If `cf_list` is None, will return position of all Cfs.

        Args:
            cf_list (list, optional): List of cf to read positions. Defaults to None.

        Returns:
            dict: Positions (`[x, y, z, yaw]`) of CF to read
        """
        if cf_list is None:
            cf_list = []
        positions = self._services["get_positions"](cf_list=str(cf_list)).positions

        return ast.literal_eval(positions)
