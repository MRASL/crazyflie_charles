#!/usr/bin/env python

"""
To manage the flight of the swarm.

Controls the position of each CF in the swarm by publishing to /cfx/goal.

Goal of each CF computed by different nodes.
TODO: Elaborate

Notes:
    Swarm: Swarm of all CFs
    Formation: Formation in which the swarm is

Services:
    - Related to a crazyflie
        - update_params: Update a param of all CF
        - emergency: Call emergency srv
        - toggle_teleop: Toggle between manual and auto #TODO toggle_teleop?
        - land: Land all CF
        - take_off: Take off all CF
        - stop: Stop all CFs
    - Related to formation
        - next_swarm_formation: Set formation to next one
        - prev_swarm_formation: Set formation to previous one

Subscribed services:
    - From /formation_manager
        - set_formation
        - update_swarm_goal
        - get_formations_list
    - From /cfx_controller
        - emergency
        - take_off
        - land
        - hover
        - stop
        - toggle_teleop

Subscription:
    - /joy_swarm_vel
    - /cfx/pose #TODO: Add pose of each CF of swarm
    - /cfx/formation_goal

Publishing:
    - /cfx/goal: Goal of CF
    - /formation_goal_vel: Velocity of formation center

"""

import rospy

from geometry_msgs.msg import Twist
from std_srvs import srv
from crazyflie_charles.srv import PoseRequest, SetFormation, GetFormationList
from crazyflie_driver.msg import Position

TAKE_OFF_DZ = 0.5 #: (float) Take off height in meters
GND_HEIGHT = 0.2 #: (float) Height of the ground

PRINT_SRV_WAIT = False

class Swarm(object):
    """Controls the swarm """

    def __init__(self, cf_list, to_sim):
        """
        Args:
            cf_list (list of str): Name of all CFs in the swarm
            to_sim (bool): To simulate or not
        """
        self.crazyflies = {} #: dict: Keys are name of the CF

        self._to_sim = to_sim   #: bool: True if in simulation
        self._to_teleop = False #: bool: If in teleop or not
        self.rate = rospy.Rate(100) #: Rate: Rate to publish messages
        self.cf_list = cf_list #: List of str: List of all cf names in the swarm

        self.joy_swarm_vel = Twist() #: Twist: Velocity received from joystick

        # Initialize each Crazyflie
        for each_cf in cf_list:
            self._init_cf(each_cf)

        # Services
        # TODO: #14 Update all parameters
        rospy.Service('/update_params', srv.Empty, self.update_params) # Update all CFs params
        rospy.Service('/emergency', srv.Empty, self.emergency)         # Emergency
        rospy.Service('/stop', srv.Empty, self.stop)                   # Stop all CFs
        rospy.Service('/take_off', srv.Empty, self.take_off)           # Take off all CFs
        rospy.Service('/land', srv.Empty, self.land)                   # Land all CFs
        rospy.Service('/toggle_teleop', srv.Empty, self.toggle_teleop) # Toggle teleop
        rospy.Service('/next_swarm_formation', srv.Empty, self.next_swarm_formation) # Formation
        rospy.Service('/prev_swarm_formation', srv.Empty, self.prev_swarm_formation) # Formation

        # Link formation manager services
        rospy.loginfo("Swarm: waiting for formation services")
        rospy.wait_for_service("set_formation")
        self.set_formation = rospy.ServiceProxy("/set_formation", SetFormation)

        rospy.wait_for_service("update_swarm_goal")
        self.update_swarm_goal = rospy.ServiceProxy("/update_swarm_goal", srv.Empty)

        rospy.wait_for_service("get_formations_list")
        self.get_formations_list = rospy.ServiceProxy("/get_formations_list", GetFormationList)
        rospy.loginfo("Swarm: formation services found")

        # Publisher
        self.formation_goal_vel_pub =\
            rospy.Publisher('/formation_goal_vel', Twist, queue_size=1)
        self.formation_goal_vel = Twist()

        # Subscribe
        rospy.Subscriber("/joy_swarm_vel", Twist, self.joy_swarm_vel_handler)

        # Find all possible formations and initialize swarm to 'line'
        self.formation_list = self.get_formations_list().formations.split(',')
        self.formation = "line"
        self.set_formation(self.formation)

    # CF initialization
    def _init_cf(self, cf_id):
        """Initialize each CF

        Create a dict /w the CF services, publishers and messages

        Args:
            cf_id (str): Name of the CF

        Note:
            Dict for all the cf and their functions/parameters
            - Keys: cf_names
                - Dict:
                    - cf: Crazyflie instance
                    - emergency service
                    - goal_publisher
                    - initial_pose
        """
        # TODO: add subdivision? i.e: self.crazyflies['cf1']['srv']['emergency] Or use a class?
        self.crazyflies[cf_id] = {"emergency": None,            # service
                                  "take_off": None,             # service
                                  "hover": None,                # service
                                  "land": None,                 # service
                                  "stop": None,                 # service
                                  "toggle_teleop": None,        # service
                                  "initial_pose": None,         # attr
                                  "formation_goal_msg": None,   # msg
                                  "goal_msg": None,             # msg
                                  "goal_pub": None,}            # publisher

        # Subscribe to services
        if not TO_SIM:
            self._link_service(cf_id, "emergency", srv.Empty)

        else:
            pass

        rospy.loginfo("Swarm: waiting services of %s " % cf_id)
        self._link_service(cf_id, "take_off", srv.Empty)
        self._link_service(cf_id, "land", srv.Empty)
        self._link_service(cf_id, "hover", srv.Empty)
        self._link_service(cf_id, "stop", srv.Empty)
        self._link_service(cf_id, "toggle_teleop", srv.Empty)
        rospy.loginfo("Swarm: found services of %s " % cf_id)

        # CF goal
        self.crazyflies[cf_id]["goal_msg"] = Position()
        self.crazyflies[cf_id]["goal_pub"] =\
            rospy.Publisher('/' + cf_id + '/goal', Position, queue_size=1)

        self.crazyflies[cf_id]["formation_goal_msg"] = Position()
        rospy.Subscriber("/%s/formation_goal" % cf_id, Position,
                         self.cf_formation_goal_handler, cf_id)

    def _link_service(self, cf_id, service_name, service_type):
        """Add a service to the dict of CFs

        Args:
            cf_id (str): Name of the CF
            service_name (str): Name of the serviec
            service_type (_): Type of the service
        """
        # rospy.loginfo("Swarm: waiting for %s service of %s " % (service_name, cf_id))
        rospy.wait_for_service('/%s/%s' % (cf_id, service_name))
        # rospy.loginfo("Swarm: found %s service of %s" % (service_name, cf_id))
        self.crazyflies[cf_id][service_name] =\
            rospy.ServiceProxy('/%s/%s' % (cf_id, service_name), service_type)

    def cf_formation_goal_handler(self, cf_formation_goal, cf_id):
        """Update formation goal of a CF

        Args:
            cf_formation_goal (Position): Goal of CF in formation
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id]["formation_goal_msg"] = cf_formation_goal

        # TODO cf_goal not always formation goal
        self.crazyflies[cf_id]["goal_msg"] = self.crazyflies[cf_id]["formation_goal_msg"]

    # Setter & getters
    def in_teleop(self):
        """
        Returns:
            bool: In teleop
        """
        return self._to_teleop

    # Publisher and subscription
    def joy_swarm_vel_handler(self, joy_swarm_vel):
        """Update swarm goal velocity

        Args:
            swarm_goal_vel (Twist): Swarm goal velocity
        """
        self.joy_swarm_vel = joy_swarm_vel

        # TODO if in formation state
        self.formation_goal_vel = self.joy_swarm_vel

    def pub_formation_goal_vel(self, formation_goal_vel):
        """Publish new swarm_goal speed

        Args:
            goal_spd (Twist): Speed variation of goal
        """
        self.formation_goal_vel_pub.publish(formation_goal_vel)

    def pub_cf_goals(self):
        """Publish goal of each CF
        """
        for _, each_cf in self.crazyflies.items():
            goal_msg = each_cf["goal_msg"]
            each_cf["goal_pub"].publish(goal_msg)

    # Services methods
    def _call_all_cf_service(self, service_name, service_msg=None):
        """Call a service for all the CF in the swarm

        Args:
            service_name (str): Name of the service to call
            service_msg (srv_msg, optional): Message to send. Defaults to None.

        Returns:
            srv_res: Response of the service
        """
        for _, each_cf in self.crazyflies.items():
            if service_msg is None:
                res = each_cf[service_name]()
            else:
                res = each_cf[service_name](service_msg)
        return res

    def toggle_teleop(self, _):
        """Toggle between manual and automatic mode

        Args:
            req (Empty): Empty

        Returns:
            EmptyResponse: Empty
        """
        self._to_teleop = not self._to_teleop
        self._call_all_cf_service("toggle_teleop")
        return srv.EmptyResponse()

    def update_params(self, _):
        """Update parameter of all swarm

        Args:
            req ([type]): [description]

        Returns:
            [type]: [description]
        """
        rospy.loginfo("Swarm: Update params")
        return srv.EmptyResponse()

    def emergency(self, _):
        """Call emergency service """
        rospy.logerr("Swarm: EMERGENCY")
        self._call_all_cf_service("emergency")
        return srv.EmptyResponse()

    def stop(self, _):
        """Call stop service """
        rospy.loginfo("Swarm: stop")
        self._call_all_cf_service("stop")
        return srv.EmptyResponse()

    def take_off(self, _):
        """Take off all cf in swarm
        """
        rospy.loginfo("Swarm: take off")

        self.update_swarm_goal()
        goal_var = Twist()

        if self.formation == "pyramid":
            goal_var.linear.z = 1.5

        else:
            goal_var.linear.z = 0.5

        self.pub_formation_goal_vel(goal_var)

        self._call_all_cf_service("take_off")
        return srv.EmptyResponse()

    def land(self, _):
        """Land all cf in swarm
        """
        rospy.loginfo("Swarm: land")

        # TODO: Land service
        # self.update_swarm_goal()
        # goal_var = Twist()
        # goal_var.linear.z = GND_HEIGHT - self.swarm_goal.z
        # self.pub_swarm_goal_vel(goal_var)

        self._call_all_cf_service("land")
        return srv.EmptyResponse()

    def next_swarm_formation(self, _):
        """Change swarm formation to next the next one
        """
        idx = self.formation_list.index(self.formation)

        next_idx = idx + 1
        if idx == (len(self.formation_list) - 1):
            next_idx = 0

        self.formation = self.formation_list[next_idx]
        self.set_formation(self.formation)
        return {}

    def prev_swarm_formation(self, _):
        """Change swarm formation to the previous one
        """
        idx = self.formation_list.index(self.formation)

        prev_idx = idx - 1
        if prev_idx < 0:
            prev_idx = len(self.formation_list) - 1

        self.formation = self.formation_list[prev_idx]
        self.set_formation(self.formation)
        return {}

    # Main function:
    def control_swarm(self):
        """Publish on topics depending of current state
        """
        self.pub_formation_goal_vel(self.formation_goal_vel)
        self.pub_cf_goals()
        self.rate.sleep()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarm_manager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')

    # Get params
    CF_LIST = rospy.get_param("~cf_list", "['cf1']")
    TO_SIM = rospy.get_param("~to_sim", "False")

    # Initialize swarm
    SWARM = Swarm(CF_LIST, TO_SIM)

    while not rospy.is_shutdown():
        SWARM.control_swarm()
