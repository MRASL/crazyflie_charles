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
    - /cfx/pose
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
import numpy as np
from numpy import dot
from numpy.linalg import norm, inv

from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import String
from crazyflie_charles.srv import SetFormation, GetFormationList, SetPositions
from crazyflie_driver.msg import Position
from state_machine import StateMachine
from crazyflie import yaw_from_quat

TAKE_OFF_DZ = 1.0 #: (float) Take off height in meters
GND_HEIGHT = 0.2 #: (float) Height of the ground

MIN_CF_DIST = 0.36
MIN_GOAL_DIST = 0.40

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
        rospy.Service('/update_swarm_params', Empty, self._update_swarm_params_srv)
        rospy.Service('/swarm_emergency', Empty, self._swarm_emergency_srv)
        rospy.Service('/stop_swarm', Empty, self._stop_swarm_srv)
        rospy.Service('/take_off_swarm', Empty, self._take_off_swarm_srv)
        rospy.Service('/land_swarm', Empty, self._land_swarm_srv)

        rospy.Service('/toggle_teleop', Empty, self._toggle_teleop_srv)
        rospy.Service('/next_swarm_formation', Empty, self._next_swarm_formation_srv)
        rospy.Service('/prev_swarm_formation', Empty, self._prev_swarm_formation_srv)
        rospy.Service('/inc_swarm_scale', Empty, self._inc_swarm_scale_srv)
        rospy.Service('/dec_swarm_scale', Empty, self._dec_swarm_scale_srv)

        rospy.Service('/traj_found', SetBool, self._traj_found_srv)
        rospy.Service('/traj_done', Empty, self._traj_done_srv)   # Trajectory done

        # Formation manager services
        rospy.loginfo("Swarm: waiting for formation services")
        rospy.wait_for_service("/set_formation")
        self.set_formation = rospy.ServiceProxy("/set_formation", SetFormation)

        rospy.wait_for_service("/get_formations_list")
        self.get_formations_list = rospy.ServiceProxy("/get_formations_list", GetFormationList)

        rospy.wait_for_service("/formation_inc_scale")
        self.inc_formation_scale = rospy.ServiceProxy("/formation_inc_scale", Empty)

        rospy.wait_for_service("/formation_dec_scale")
        self.dec_formation_scale = rospy.ServiceProxy("/formation_dec_scale", Empty)

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
        self.extra_cf_list = [] #: list of str: ID of extra CF
        self.landed_cf_ids = [] #: list of str: Swarm Id of landed CFs

        # Initialize state machine
        self.state_list = {"landed": self.swarm_landed,
                           "follow_traj": self.follow_traj,
                           "in_formation": self.swarm_in_formation,
                           "hover": self.hover_swarm,
                           "land_swarm": self.land_swarm_state,}
        self.state_machine = StateMachine(self.state_list)
        self.state_machine.set_state("landed")
        self.next_state = ""
        self.traj_found = False
        self.traj_successfull = False

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
        self.crazyflies[cf_id] = {"emergency": None,            # service
                                  "take_off": None,             # service
                                  "hover": None,                # service
                                  "land": None,                 # service
                                  "stop": None,                 # service
                                  "toggle_teleop": None,        # service
                                  "pose": None,                 # attr
                                  "initial_pose": None,         # attr
                                  "state": None,                # attr
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
        self._link_service(cf_id, "emergency", Empty)

        rospy.loginfo("Swarm: found services of %s " % cf_id)

        # CF pose
        self.crazyflies[cf_id]["pose"] = None
        rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self.cf_pose_handler, cf_id)

        # Wait to receive first position
        while self.crazyflies[cf_id]["pose"] is None:
            pass

        # CF goal
        self.crazyflies[cf_id]["goal_msg"] = Position()
        self.crazyflies[cf_id]["goal_msg"].x = self.crazyflies[cf_id]["pose"].pose.position.x
        self.crazyflies[cf_id]["goal_msg"].y = self.crazyflies[cf_id]["pose"].pose.position.y
        self.crazyflies[cf_id]["goal_msg"].z = self.crazyflies[cf_id]["pose"].pose.position.z

        self.crazyflies[cf_id]["goal_pub"] =\
            rospy.Publisher('/' + cf_id + '/goal', Position, queue_size=1)

        self.crazyflies[cf_id]["formation_goal_msg"] = Position()
        rospy.Subscriber("/%s/formation_goal" % cf_id, Position,
                         self.cf_formation_goal_handler, cf_id)

        # CF traj goal
        rospy.Subscriber("/%s/trajectory_goal" % cf_id, Position, self.cf_traj_goal_handler, cf_id)

        # CF state
        rospy.Subscriber("/%s/state" % cf_id, String, self.cf_state_handler, cf_id)

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
        self.crazyflies[cf_id]["formation_goal_msg"] = cf_formation_goal

    def cf_state_handler(self, cf_state, cf_id):
        """Update state of a CF

        Args:
            cf_state (String): Current state of CF state machine
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id]["state"] = cf_state.data

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
    def _call_all_cf_service(self, service_name, service_msg=None, cf_list=None):
        """Call a service for all the CF in the swarm

        Args:
            service_name (str): Name of the service to call
            service_msg (srv_msg, optional): Message to send. Defaults to None.
            cf_list (list of str, optional): Only call service of CF in list. Defaults to None.
        """
        for cf_id, each_cf in self.crazyflies.items():
            if cf_list is None:
                if service_msg is None:
                    each_cf[service_name]()
                else:
                    each_cf[service_name](service_msg)

            elif cf_id in cf_list:
                if service_msg is None:
                    each_cf[service_name]()
                else:
                    each_cf[service_name](service_msg)

        return {}

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

    # Swarm states
    def _update_swarm_params_srv(self, _):
        """Update parameter of all swarm

        Args:
            req ([type]): [description]

        Returns:
            [type]: [description]
        """
        rospy.loginfo("Swarm: Update params")
        return {}

    def _swarm_emergency_srv(self, _):
        """Call emergency service """
        rospy.logerr("Swarm: EMERGENCY")
        self._call_all_cf_service("emergency")
        return {}

    def _stop_swarm_srv(self, _):
        """Call stop service """
        rospy.loginfo("Swarm: stop")
        self._call_all_cf_service("stop")
        return {}

    def _take_off_swarm_srv(self, _):
        """Take off all cf in swarm
        """
        self.send_formation()

        self.go_to_formation()
        return {}

    def _land_swarm_srv(self, _):
        """Land all cf in swarm
        """
        self.go_to_formation(True)
        return {}

    def _follow_traj_srv(self, _):
        """Follow a trajectory
        """
        self.state_machine.set_state("follow_traj")
        return {}

    # Formation
    def send_formation(self):
        """Set swarm formation and find CFs in extra
        """
        start_positions = {}
        for cf_id, cf_vals in self.crazyflies.items():
            cf_pose = cf_vals["pose"].pose

            start_positions[cf_id] = [cf_pose.position.x,
                                      cf_pose.position.y,
                                      cf_pose.position.z]

        srv_res = self.set_formation(formation=self.formation, positions=str(start_positions))

        self.extra_cf_list = srv_res.extra_cf.split(',')

    def update_formation(self):
        """Update formation of formation manager to match current formation
        """
        # Change state to make sure CF don't 'teleport' to new formation
        self.state_machine.set_state("hover")
        rospy.sleep(0.1)

        # Set new formation
        self.send_formation()

        self.go_to_formation()

    def _next_swarm_formation_srv(self, _):
        """Change swarm formation to next the next one
        """
        if self.state_machine.in_state("in_formation"):
            idx = self.formation_list.index(self.formation)

            next_idx = idx + 1
            if idx == (len(self.formation_list) - 1):
                next_idx = 0

            self.formation = self.formation_list[next_idx]
            self.update_formation()

        return {}

    def _prev_swarm_formation_srv(self, _):
        """Change swarm formation to the previous one
        """
        if self.state_machine.in_state("in_formation"):
            idx = self.formation_list.index(self.formation)

            prev_idx = idx - 1
            if prev_idx < 0:
                prev_idx = len(self.formation_list) - 1

            self.formation = self.formation_list[prev_idx]
            self.update_formation()

        return {}

    def _inc_swarm_scale_srv(self, _):
        """Increase formation scale
        """
        if self.state_machine.in_state("in_formation"):
            # Change state to make sure CF don't 'teleport' to new formation
            self.state_machine.set_state("hover")
            rospy.sleep(0.1)

            self.inc_formation_scale()

            self.go_to_formation()

        return {}

    def _dec_swarm_scale_srv(self, _):
        """Decrease formation scale
        """
        if self.state_machine.in_state("in_formation"):
            # Change state to make sure CF don't 'teleport' to new formation
            self.state_machine.set_state("hover")
            rospy.sleep(0.1)

            self.dec_formation_scale()

            self.go_to_formation()

        return {}

    # Trajectory
    def _traj_found_srv(self, srv_req):
        """Called when trajectory solver is done.

        Args:
            srv_req.data (bool): True if a non colliding trajectory is found
        """
        self.traj_found = True
        self.traj_successfull = srv_req.data

        if not self.traj_successfull:
            rospy.logerr("Swarm: No trajectory found")

        return {'success': True, "message": ""}

    def _traj_done_srv(self, _):
        """Called when trajectory has all been executed.

        Goes to formation?
        """
        self.state_machine.set_state(self.next_state)
        return {}

    # Methods to change between formations/states
    def go_to_formation(self, land_swarm=False):
        """Controls swarm to reach desired formation while avoiding collisions

        Handles extra CF landing

        Args:
            land_swarm (bool, optional): If true, land swarm to initial pos. Defaults to False.
        """
        rospy.loginfo("Swarm: Going to new formation")
        # rospy.loginfo("Swarm: CF in extra: {}".format(self.extra_cf_list))

        self.traj_found = False

        self.landed_cf_ids = []
        rospy.sleep(0.1)

        # Update goal and send positions to planner
        self.update_cf_goal(land_swarm)
        self.send_start_positions()
        self.send_goals(land_swarm)

        # Start solver
        self.start_trajectory_planner()

        # Take off landed CF that are not in extra
        take_off_list = [cf for cf in self.landed_cf_ids if cf not in self.extra_cf_list]
        # rospy.loginfo("CF to take off: {}".format(take_off_list))
        self._call_all_cf_service("take_off", cf_list=take_off_list)

        self.wait_for_take_off()

        self.wait_for_planner()

        # Handle planner result
        if self.traj_successfull:
            if land_swarm:
                self.next_state = "land_swarm"
            else:
                self.next_state = "in_formation"
            self.start_trajectory_pub() # Starting trajectory publisher
            self.state_machine.set_state("follow_traj")

        else: # If no trajectory is found, go back to start # TODO: Better fault handling
            # self.state_machine.set_state("land_swarm")
            self.state_machine.set_state("hover")

    def update_cf_goal(self, land_swarm):
        """Update goal and initial position of landed CFs

        Also find Id of landed CFs

        Notes:
            - If land_swarm: Set all goals 0.5m above initial pose
            - If landed and not in extra: Set goal 0.5m above initial pose and add to landed list
            - If landed and in extra: Set goal to initial pose and add to landed list

        Args:
            land_swarm (bool): When True, land all CFs in swarm
        """
        for cf_id, cf_vals in self.crazyflies.items():
            if land_swarm:
                cf_initial_pose = cf_vals["initial_pose"]

                cf_vals["goal_msg"].x = cf_initial_pose.position.x
                cf_vals["goal_msg"].y = cf_initial_pose.position.y
                cf_vals["goal_msg"].z = cf_initial_pose.position.z + 0.5

            # If CF is landed and not in extra
            elif cf_vals["state"] in ["stop", "landed", "land"] and cf_id not in self.extra_cf_list:
                self.landed_cf_ids.append(cf_id)
                cf_pose = cf_vals["pose"].pose

                if cf_vals["initial_pose"] is None:
                    cf_vals["initial_pose"] = cf_pose

                cf_vals["goal_msg"].x = cf_pose.position.x
                cf_vals["goal_msg"].y = cf_pose.position.y
                cf_vals["goal_msg"].z = cf_pose.position.z + 0.5

            # If CF is landed and in extra
            elif cf_vals["state"] in ["stop", "landed", "land"] and cf_id in self.extra_cf_list:
                self.landed_cf_ids.append(cf_id)
                cf_pose = cf_vals["pose"].pose

                if cf_vals["initial_pose"] is None:
                    cf_vals["initial_pose"] = cf_pose

                cf_vals["goal_msg"].x = cf_pose.position.x
                cf_vals["goal_msg"].y = cf_pose.position.y
                cf_vals["goal_msg"].z = cf_pose.position.z

    def send_start_positions(self):
        """Send start positions to trajectory planner

        Notes:
            - If CF is landed, starting position is considered 0.5m above
            - Doesn't send positon of extra CF if it's landed
        """
        # Send each CF starting position to planner
        start_positions = {}
        for cf_id, cf_vals in self.crazyflies.items():
            cf_pose = cf_vals["pose"].pose

            # If landed, starting position is 0.5m above
            if cf_id in self.landed_cf_ids and cf_id not in self.extra_cf_list:
                start_positions[cf_id] = [cf_pose.position.x,
                                          cf_pose.position.y,
                                          cf_pose.position.z + 0.5,
                                          yaw_from_quat(cf_pose.orientation)]

            else:
                start_positions[cf_id] = [cf_pose.position.x,
                                          cf_pose.position.y,
                                          cf_pose.position.z,
                                          yaw_from_quat(cf_pose.orientation)]

        self.set_planner_positions(position_type="start_position", positions=str(start_positions))

    def send_goals(self, land_swarm):
        """Send goal of each CF to trajectory planner

        Notes:
            - If CF is in extra, goal is 0.5m above initial position
            - If CF is in extra and landed, no goal is sent

        Args:
            land_swarm (bool): If True, land all CF in the swarm
        """
        goals = {}
        for cf_id, cf_vals in self.crazyflies.items():
            # Land all CFs in air
            if land_swarm and cf_id not in self.landed_cf_ids:
                cf_initial_pose = cf_vals["initial_pose"]
                goals[cf_id] = [cf_initial_pose.position.x,
                                cf_initial_pose.position.y,
                                cf_initial_pose.position.z + 0.5,
                                yaw_from_quat(cf_initial_pose.orientation),]

            # Goal of CF in formation
            elif cf_id not in self.extra_cf_list: # If CF in formation
                formation_goal = cf_vals["formation_goal_msg"]
                goals[cf_id] = [formation_goal.x,
                                formation_goal.y,
                                formation_goal.z,
                                formation_goal.yaw]

            # If CF in extra and not landed, go to initial position
            elif cf_id not in self.landed_cf_ids:
                cf_initial_pose = cf_vals["initial_pose"]
                goals[cf_id] = [cf_initial_pose.position.x,
                                cf_initial_pose.position.y,
                                cf_initial_pose.position.z + 0.5,
                                yaw_from_quat(cf_initial_pose.orientation),]

        self.set_planner_positions(position_type="goal", positions=str(goals))

    def wait_for_take_off(self):
        """Wait that all CF of formation are in hover state
        """
        all_cf_in_air = False #: bool: True if all CFs are done taking off
        while not all_cf_in_air:
            all_cf_in_air = True

            if rospy.is_shutdown():
                break

            for cf_id, each_cf in self.crazyflies.items():
                if each_cf["state"] != "hover" and cf_id not in self.extra_cf_list:
                    all_cf_in_air = False

            self.rate.sleep()

    def wait_for_planner(self):
        """Wait until planner finds a trajectory
        """
        rospy.sleep(0.1) # Make sure first traj messages are received
        while not self.traj_found:
            if rospy.is_shutdown():
                break

            for _, each_cf in self.crazyflies.items():
                each_cf["goal_msg"] = each_cf["goal_msg"]

            self.rate.sleep()

        self.traj_found = False

    def update_cf_goal_to_land(self, land_swarm=False):
        """Set goal of CF to land to initial position

        Args:
            land_swarm (bool, optional): True if all swarm is to be landed. Default: False
        """
        for cf_id, cf_vals in self.crazyflies.items():
            # If land all swarm or (cf in extra and not landed)
            if land_swarm or\
                            (cf_vals["state"] not in ["landed", "land", "stop"] and\
                            cf_id in self.extra_cf_list):

                cf_initial_pose = cf_vals["initial_pose"]

                cf_vals["goal_msg"].x = cf_initial_pose.position.x
                cf_vals["goal_msg"].y = cf_initial_pose.position.y
                cf_vals["goal_msg"].z = cf_initial_pose.position.z
                # cf_vals["goal_msg"].yaw = yaw_from_quat(cf_initial_pose.orientation)

    # State methods
    def swarm_landed(self):
        """All CF are on the ground
        """
        pass

    def follow_traj(self):
        """All cf follow a specified trajectory
        """
        # Set CF goal to trajectory goal
        for cf_id, each_cf in self.crazyflies.items():
            traj_goal = each_cf["traj_goal_msg"]
            # Make sure first msg was sent
            if traj_goal is not None:
                #If CF in extra and landed, don't follow goal
                if each_cf["state"] in ["landed", "land", "stop"] and cf_id in self.extra_cf_list:
                    each_cf["goal_msg"] = each_cf["goal_msg"]
                else:
                    each_cf["goal_msg"] = traj_goal

    def swarm_in_formation(self):
        """Swarm is in a specific formation. Lands extra CFs

        Formation can be moved /w the joystick
        """
        # Publish goal velocity
        self.formation_goal_vel = self.joy_swarm_vel
        self.pub_formation_goal_vel(self.formation_goal_vel)

        # Set CF goal to formation goal
        for cf_id, each_cf in self.crazyflies.items():
            # If CF part of formation, set goal to formation_goal
            if cf_id not in self.extra_cf_list:
                each_cf["goal_msg"] = each_cf["formation_goal_msg"]

            # IF CF is not landed
            elif each_cf["state"] not in ["landed", "land", "stop"]:
                self.update_cf_goal_to_land()
                self._call_all_cf_service("land", cf_list=self.extra_cf_list)

            # If CF is already landed
            else:
                each_cf["goal_msg"] = each_cf["goal_msg"]

    def hover_swarm(self):
        """CFs hover in place
        """
        # Set CF goal to current pose
        for _, each_cf in self.crazyflies.items():
            each_cf["goal_msg"] = each_cf["goal_msg"]

    def land_swarm_state(self):
        """Land CF to their starting position
        """
        rospy.loginfo("Swarm: land")
        self.update_cf_goal_to_land(True)
        self._call_all_cf_service("land")
        self.state_machine.set_state("landed")

    # Execute state
    def check_positions(self):
        """Make sure position and goal of each CF respect min distance
        """
        position_dist = []
        goal_dist = []

        scaling_matrix_inv = inv(np.diag([1, 1, 2]))

        for cf_id, cf_vals in self.crazyflies.items():

            cf_position = cf_vals["pose"].pose
            cf_position = np.array([cf_position.position.x,
                                    cf_position.position.y,
                                    cf_position.position.z])

            cf_goal = cf_vals["goal_msg"]
            cf_goal = np.array([cf_goal.x,
                                cf_goal.y,
                                cf_goal.z])

            for other_id, other_vals in self.crazyflies.items():
                if other_id != cf_id:
                    other_pos = other_vals["pose"].pose
                    other_pos = np.array([other_pos.position.x,
                                          other_pos.position.y,
                                          other_pos.position.z])

                    other_goal = other_vals["goal_msg"]
                    other_goal = np.array([other_goal.x,
                                           other_goal.y,
                                           other_goal.z])

                    p_dist = norm(dot(scaling_matrix_inv, cf_position - other_pos))
                    g_dist = norm(dot(scaling_matrix_inv, cf_goal - other_goal))

                    position_dist.append(p_dist)
                    goal_dist.append(g_dist)

        min_pos_dist = min(position_dist)
        min_goal_dist = min(goal_dist)

        print "Min distances:"
        print "\t position: %.2f" % min_pos_dist
        print "\t goal: %.2f" % min_goal_dist

        if min_goal_dist < MIN_GOAL_DIST:
            rospy.logwarn("Goals are too close")

        if min_pos_dist < MIN_CF_DIST:
            rospy.logerr("CF too close, emergency")
            self._swarm_emergency_srv(Empty())

    def control_swarm(self):
        """Publish on topics depending of current state
        """
        # Execute function depending on current state
        state_function = self.state_machine.run_state()
        state_function()

        self.check_positions()

        # Publish goal of each CF
        self.pub_cf_goals()

        self.rate.sleep()


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
