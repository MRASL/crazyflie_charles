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

Etapes generales:
    1 - Initialisation
        - [x] Setup des CF
        - [x] Setup de la formation
            - Goal initial
            - Goal de chaque CF
        - [ ] Setup traj. planner
    2 - Landed
        - [x] Position initiale de chaque CF
    3 - Take off
        - [x] Decolle de x m au dessus de pos initiale
    4 - Go to formation
        - [ ] Associe chaque CF a une pos
        - [ ] Plan trajectoire de chaque CF
        - [ ] Suit la trajectoire
    5 - En formation
        - [x] Suit commandes du joystick
    7 - Change formation
        - [x] Compute nouveaux goals
        - [ ] Compute trajectoires de chaque CF
        - [ ] Va au nouveau but
    8 - Chage scale
        - [x] Compute nouveaux goals
        - [ ] Compute trajectoires de chaque CF
        - [ ] Va au nouveau but
    9 - Land
        - [ ] Compute traj jusqu'a x m de la position initiale
        - [ ] Chaque CF va au dessus de sa position initiale
        - [x] Land
"""

import rospy

from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, SetBool
from crazyflie_charles.srv import SetFormation, GetFormationList, SetPositions
from crazyflie_driver.msg import Position
from state_machine import StateMachine

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
        rospy.loginfo("Swarm: Initialization...")

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
        rospy.Service('/update_params', Empty, self._update_params_srv) # Update all CFs params
        rospy.Service('/emergency', Empty, self._emergency_srv)         # Emergency
        rospy.Service('/stop', Empty, self._stop_srv)                   # Stop all CFs
        rospy.Service('/take_off', Empty, self._take_off_srv)           # Take off all CFs
        rospy.Service('/land', Empty, self._land_srv)                   # Land all CFs
        rospy.Service('/hover', Empty, self._hover_srv)                 # Hover state
        rospy.Service('/follow_traj', Empty, self._follow_traj_srv)     # Follow traj state
        rospy.Service('/in_formation', Empty, self._in_formation_srv)   # Go to in formation state
        rospy.Service('/toggle_teleop', Empty, self._toggle_teleop_srv) # Toggle teleop
        rospy.Service('/next_swarm_formation', Empty, self._next_swarm_formation_srv)# Formation
        rospy.Service('/prev_swarm_formation', Empty, self._prev_swarm_formation_srv)# Formation
        rospy.Service('/traj_found', SetBool, self._traj_found_srv)

        # Formation manager services
        rospy.loginfo("Swarm: waiting for formation services")
        rospy.wait_for_service("set_formation")
        self.set_formation = rospy.ServiceProxy("/set_formation", SetFormation)

        rospy.wait_for_service("get_formations_list")
        self.get_formations_list = rospy.ServiceProxy("/get_formations_list", GetFormationList)
        rospy.loginfo("Swarm: formation services found")

        # Trajectory planner services
        rospy.loginfo("Swarm: waiting for trajectory planner services")
        self.set_planner_positions = rospy.ServiceProxy("/set_planner_positions", SetPositions)
        self.start_trajectory_planner = rospy.ServiceProxy("/plan_trajectories", Empty)
        self.start_trajectory_pub = rospy.ServiceProxy("/pub_trajectories", Empty)
        rospy.loginfo("Swarm: trajectory planner services found")

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

        # Initialize state machine
        self.state_list = {"landed": self.landed_state,
                           "take_off":self.take_off_state,
                           "follow_traj": self.follow_traj_state,
                           "in_formation": self.in_formation_state,
                           "hover": self.hover_state,
                           "land": self.land_state,}
        self.state_machine = StateMachine(self.state_list)
        self.state_machine.set_state("landed")
        self.traj_found = False

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
                                  "pose": None,                 # attr
                                  "initial_pose": None,         # attr
                                  "formation_goal_msg": None,   # msg
                                  "traj_goal_msg": None,        # msg
                                  "goal_msg": None,             # msg
                                  "goal_pub": None,}            # publisher

        # Subscribe to services
        rospy.loginfo("Swarm: waiting services of %s " % cf_id)
        self._link_service(cf_id, "take_off", Empty)
        self._link_service(cf_id, "land", Empty)
        self._link_service(cf_id, "hover", Empty)
        self._link_service(cf_id, "stop", Empty)
        self._link_service(cf_id, "toggle_teleop", Empty)

        if not TO_SIM:
            self._link_service(cf_id, "emergency", Empty)
        else:
            pass

        rospy.loginfo("Swarm: found services of %s " % cf_id)

        # CF pose
        self.crazyflies[cf_id]["pose"] = PoseStamped()
        rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self.cf_pose_handler, cf_id)

        # CF goal
        self.crazyflies[cf_id]["goal_msg"] = Position()
        self.crazyflies[cf_id]["goal_pub"] =\
            rospy.Publisher('/' + cf_id + '/goal', Position, queue_size=1)

        self.crazyflies[cf_id]["formation_goal_msg"] = Position()
        rospy.Subscriber("/%s/formation_goal" % cf_id, Position,
                         self.cf_formation_goal_handler, cf_id)

        # CF traj goal
        rospy.Subscriber("/%s/trajectory_goal" % cf_id, Position, self.cf_traj_goal_handler, cf_id)

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

    def cf_pose_handler(self, pose_stamped, cf_id):
        """Update current position of a cf

        Args:
            pose_stamped (PoseStamped): New pose of CF
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id]["pose"] = pose_stamped

    def cf_traj_goal_handler(self, traj_goal, cf_id):
        """Update trajectory goal of CF

        Args:
            traj_goal (Position): New pose of CF
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id]["traj_goal_msg"] = traj_goal

    def cf_formation_goal_handler(self, cf_formation_goal, cf_id):
        """Update formation goal of a CF

        Args:
            cf_formation_goal (Position): Goal of CF in formation
            cf_id (int): Id of the CF
        """
        self.first_traj_msg = True
        self.crazyflies[cf_id]["formation_goal_msg"] = cf_formation_goal

    def pub_formation_goal_vel(self, formation_goal_vel):
        """Publish swarm_goal speed

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

    def _toggle_teleop_srv(self, _):
        """Toggle between manual and automatic mode

        Args:
            req (Empty): Empty

        Returns:
            EmptyResponse: Empty
        """
        self._to_teleop = not self._to_teleop
        self._call_all_cf_service("toggle_teleop")
        return {}

    def _update_params_srv(self, _):
        """Update parameter of all swarm

        Args:
            req ([type]): [description]

        Returns:
            [type]: [description]
        """
        rospy.loginfo("Swarm: Update params")
        return {}

    def _emergency_srv(self, _):
        """Call emergency service """
        rospy.logerr("Swarm: EMERGENCY")
        self._call_all_cf_service("emergency")
        return {}

    def _stop_srv(self, _):
        """Call stop service """
        rospy.loginfo("Swarm: stop")
        self._call_all_cf_service("stop")
        return {}

    def _take_off_srv(self, _):
        """Take off all cf in swarm
        """
        self.state_machine.set_state("take_off")
        return {}

    def _land_srv(self, _):
        """Land all cf in swarm
        """
        self.state_machine.set_state("land")
        return {}

    def _hover_srv(self, _):
        """Go to hover state
        """
        self.state_machine.set_state("hover")
        return {}

    def _follow_traj_srv(self, _):
        """Follow a trajectory
        """
        self.state_machine.set_state("follow_traj")
        return {}

    def _in_formation_srv(self, _):
        """Go to in formation state
        """
        self.state_machine.set_state("in_formation")
        return {}

    def _next_swarm_formation_srv(self, _):
        """Change swarm formation to next the next one
        """
        idx = self.formation_list.index(self.formation)

        next_idx = idx + 1
        if idx == (len(self.formation_list) - 1):
            next_idx = 0

        self.formation = self.formation_list[next_idx]
        self.set_formation(self.formation)
        return {}

    def _prev_swarm_formation_srv(self, _):
        """Change swarm formation to the previous one
        """
        idx = self.formation_list.index(self.formation)

        prev_idx = idx - 1
        if prev_idx < 0:
            prev_idx = len(self.formation_list) - 1

        self.formation = self.formation_list[prev_idx]
        self.set_formation(self.formation)
        return {}

    def _traj_found_srv(self, srv_req):
        result = srv_req.data

        if result:
            self.traj_found = True

        else:
            rospy.logerr("Swarm: No trajectory found")

        return {'success': True, "message": ""}

    # Main functions:
    def control_swarm(self):
        """Publish on topics depending of current state
        """
        # Execute function depending on current state
        state_function = self.state_machine.run_state()
        state_function()

        # Publish goal of each CF
        self.pub_cf_goals()

        self.rate.sleep()

    def landed_state(self):
        """All CF are on the ground
        """
        pass

    def take_off_state(self):
        """Take off all CF in the swarm.
        """
        rospy.loginfo("Swarm: take off")
        self.traj_found = False

        # Update CF goal
        for cf_id, cf_vals in self.crazyflies.items():
            cf_pose = cf_vals["pose"].pose
            cf_vals["goal_msg"].x = cf_pose.position.x
            cf_vals["goal_msg"].y = cf_pose.position.y
            cf_vals["goal_msg"].z = cf_pose.position.z + 0.5

        # Send each CF starting position to planner
        start_positions = {}
        for cf_id, cf_vals in self.crazyflies.items():
            cf_pose = cf_vals["pose"].pose
            start_positions[cf_id] = [cf_pose.position.x, cf_pose.position.y, cf_pose.position.z]
        self.set_planner_positions(position_type="start_position", positions=str(start_positions))

        # Send each CF goal to planner
        goals = {}
        for cf_id, cf_vals in self.crazyflies.items():
            formation_goal = cf_vals["formation_goal_msg"]
            goals[cf_id] = [formation_goal.x, formation_goal.y, formation_goal.z]

        self.set_planner_positions(position_type="goal", positions=str(goals))

        # Start solver
        self.start_trajectory_planner()

        # Take off all CF
        self._call_all_cf_service("take_off")

        # Change state
        self.state_machine.set_state("hover")

    def follow_traj_state(self):
        """All cf follow a specified trajectory
        """
        # Set CF goal to trajectory goal
        for _, each_cf in self.crazyflies.items():
            traj_goal = each_cf["traj_goal_msg"]
            if traj_goal is not None:
                each_cf["goal_msg"] = traj_goal

    def in_formation_state(self):
        """Swarm is in a specific formation.

        Formation can be moved /w the joystick
        """
        # Publish goal velocity
        self.formation_goal_vel = self.joy_swarm_vel
        self.pub_formation_goal_vel(self.formation_goal_vel)

        # Set CF goal to formation goal
        for _, each_cf in self.crazyflies.items():
            each_cf["goal_msg"] = each_cf["formation_goal_msg"]

    def hover_state(self):
        """CFs hover in place
        """

        # Set CF goal to current pose
        for _, each_cf in self.crazyflies.items():
            each_cf["goal_msg"] = each_cf["goal_msg"]

        duration = rospy.Duration(3)
        rospy.sleep(duration)

        self.start_trajectory_pub()
        self.state_machine.set_state("follow_traj")

    def land_state(self):
        """Land CF to their starting position
        """
        rospy.loginfo("Swarm: land")
        self._call_all_cf_service("land")
        self.state_machine.set_state("landed")

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarm_manager', anonymous=False)

    # Get params
    CF_LIST = rospy.get_param("~cf_list", "['cf1']")
    TO_SIM = rospy.get_param("~to_sim", "False")

    # Initialize swarm
    SWARM = Swarm(CF_LIST, TO_SIM)

    while not rospy.is_shutdown():
        SWARM.control_swarm()
