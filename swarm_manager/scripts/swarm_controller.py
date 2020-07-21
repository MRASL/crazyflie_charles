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
from swarm_manager.srv import SetParam

from launch_swarm import launch_swarm

class SwarmController(object):
    """Python API for easy control of the swarm.
    """
    def __init__(self, cf_list, to_sim):
        """
        Args:
            cf_list (list of str): Name of all CFs in the swarm
            to_sim (bool): To simulate or not
        """
        rospy.loginfo("Swarm: Initialization...")

        self.crazyflies = {} # type: Dict[str, CrazyfliePy]
        #: Keys are name of the CF

        self._to_sim = to_sim   #: bool: True if in simulation
        self.rate = rospy.Rate(10) #: Rate: Rate to publish messages
        self.cf_list = cf_list #: List of str: List of all cf names in the swarm

        self.joy_swarm_vel = Twist() #: Twist: Velocity received from joystick

        # Initialize each Crazyflie
        for each_cf in cf_list:
            self.crazyflies[each_cf] = CrazyfliePy(each_cf)

        self._init_params()

        self.formation_services = {}
        self.formation_services = {"set_formation": None,
                                   "get_list": None,
                                   "inc_scale": None,
                                   "dec_scale": None}

        self.traj_services = {}
        self.traj_services = {"set_planner_positions": None,
                              "plan_trajectories": None,
                              "pub_trajectories": None}
        self._init_services()

        # Publisher
        self.formation_goal_vel_pub = rospy.Publisher('/formation_goal_vel',
                                                      Twist, queue_size=1)
        self.formation_goal_vel = Twist()

        # Subscribe
        rospy.Subscriber("/joy_swarm_vel", Twist, self._joy_swarm_vel_handler)


        # Find all possible formations and initialize swarm to 'line'
        self.formation_list = self.formation_services["get_list"]().formations.split(',')
        self.formation = "circle"
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

    def _init_services(self):
        # Services
        rospy.Service('/swarm_emergency', Empty, self._swarm_emergency_srv)
        rospy.Service('/stop_swarm', Empty, self._stop_swarm_srv)
        rospy.Service('/take_off_swarm', Empty, self._take_off_swarm_srv)
        rospy.Service('/land_swarm', Empty, self._land_swarm_srv)

        rospy.Service('/next_swarm_formation', Empty, self._next_swarm_formation_srv)
        rospy.Service('/prev_swarm_formation', Empty, self._prev_swarm_formation_srv)
        rospy.Service('/inc_swarm_scale', Empty, self._inc_swarm_scale_srv)
        rospy.Service('/dec_swarm_scale', Empty, self._dec_swarm_scale_srv)

        rospy.Service('/traj_found', SetBool, self._traj_found_srv)
        rospy.Service('/traj_done', Empty, self._traj_done_srv)   # Trajectory done

        # Subscribe to services
        # Formation manager
        rospy.loginfo("Swarm: waiting for formation services")
        rospy.wait_for_service("/set_formation")
        rospy.wait_for_service("/get_formations_list")
        rospy.wait_for_service("/formation_inc_scale")
        rospy.wait_for_service("/formation_dec_scale")
        self.formation_services["set_formation"] = rospy.ServiceProxy("/set_formation",
                                                                      SetFormation)
        self.formation_services["get_list"] = rospy.ServiceProxy("/get_formations_list",
                                                                 GetFormationList)
        self.formation_services["inc_scale"] = rospy.ServiceProxy("/formation_inc_scale", Empty)
        self.formation_services["dec_scale"] = rospy.ServiceProxy("/formation_dec_scale", Empty)
        rospy.loginfo("Swarm: formation services found")

        # Trajectory planner
        rospy.loginfo("Swarm: waiting for trajectory planner services")
        self.traj_services["set_planner_positions"] = rospy.ServiceProxy("/set_planner_positions",
                                                                         SetPositions)
        self.traj_services["plan_trajectories"] = rospy.ServiceProxy("/plan_trajectories", Empty)
        self.traj_services["pub_trajectories"] = rospy.ServiceProxy("/pub_trajectories", Empty)
        rospy.loginfo("Swarm: trajectory planner services found")

    def _init_params(self):
        self.update_swarm_param("commander/enHighLevel", 1)
        self.update_swarm_param("stabilizer/estimator", 2) # Use EKF
        self.update_swarm_param("stabilizer/controller", 1) # 1: High lvl, 2: Mellinger
        self.update_swarm_param("kalman/resetEstimation", 1)

    # Publisher and subscription
    def _joy_swarm_vel_handler(self, joy_swarm_vel):
        """Update swarm goal velocity

        Args:
            swarm_goal_vel (Twist): Swarm goal velocity
        """
        self.joy_swarm_vel = joy_swarm_vel

    def _pub_formation_goal_vel(self, formation_goal_vel):
        """Publish swarm_goal speed

        Args:
            goal_spd (Twist): Speed variation of goal
        """
        self.formation_goal_vel_pub.publish(formation_goal_vel)

    def _pub_cf_goals(self):
        """Publish goal of each CF
        """
        for _, each_cf in self.crazyflies.items():
            each_cf.publish_goal()

    # Services methods
    def _call_all_cf_service(self, service_name, args=None, kwargs=None, cf_list=None):
        """Call a service for all the CF in the swarm.

        If a list /w CF name is specified, will only call the srv for CF in the list.

        Args:
            service_name (str): Name of the service to call
            service_msg (srv_msg, optional): Message to send. Defaults to None.
            cf_list (list of str, optional): Only call service of CF in list. Defaults to None.
        """
        for cf_id, each_cf in self.crazyflies.items():
            if cf_list is None or cf_id in cf_list:
                each_cf.call_srv(service_name, args, kwargs)

        return {}

    # Services
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

            self.formation_services["inc_scale"]()

            self.go_to_formation()

        return {}

    def _dec_swarm_scale_srv(self, _):
        """Decrease formation scale
        """
        if self.state_machine.in_state("in_formation"):
            # Change state to make sure CF don't 'teleport' to new formation
            self.state_machine.set_state("hover")
            rospy.sleep(0.1)

            self.formation_services["dec_scale"]()

            self.go_to_formation()

        return {}

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

    # Methods
    def update_swarm_param(self, param, value):
        """Update parameter of all swarm

        Args:
            param (str): Name of parameter
            value (int64): Parameter value
        """
        rospy.loginfo("Swarm: Set %s param to %s" % (param, str(value)))
        self._call_all_cf_service("set_param", kwargs={'param': param, 'value': value})

    # Formation
    def send_formation(self):
        """Set swarm formation and find CFs in extra
        """
        start_positions = {}
        for cf_id, cf_vals in self.crazyflies.items():
            cf_pose = cf_vals.pose.pose

            start_positions[cf_id] = [cf_pose.position.x,
                                      cf_pose.position.y,
                                      cf_pose.position.z]

        srv_res = self.formation_services["set_formation"](formation=self.formation,
                                                           positions=str(start_positions))

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
        self.traj_services["plan_trajectories"]()

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
                # TODO: Trade spots
                # self.next_state = "hover"
                self.next_state = "in_formation"

            self.traj_services["pub_trajectories"]() # Starting trajectory publisher
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
                cf_initial_pose = cf_vals.initial_pose

                cf_vals.goals['goal'].x = cf_initial_pose.position.x
                cf_vals.goals['goal'].y = cf_initial_pose.position.y
                cf_vals.goals['goal'].z = cf_initial_pose.position.z + TAKE_OFF_HEIGHT

            # If CF is landed and not in extra
            elif cf_vals.state in ["stop", "landed", "land"] and cf_id not in self.extra_cf_list:
                self.landed_cf_ids.append(cf_id)
                cf_pose = cf_vals.pose.pose

                if cf_vals.initial_pose is None:
                    cf_vals.update_initial_pose()

                cf_vals.goals["goal"].x = cf_pose.position.x
                cf_vals.goals["goal"].y = cf_pose.position.y
                cf_vals.goals["goal"].z = cf_pose.position.z + TAKE_OFF_HEIGHT

            # If CF is landed and in extra
            elif cf_vals.state in ["stop", "landed", "land"] and cf_id in self.extra_cf_list:
                self.landed_cf_ids.append(cf_id)
                cf_pose = cf_vals.pose.pose

                if cf_vals.initial_pose is None:
                    cf_vals.update_initial_pose()

                cf_vals.goals["goal"].x = cf_pose.position.x
                cf_vals.goals["goal"].y = cf_pose.position.y
                cf_vals.goals["goal"].z = cf_pose.position.z

    def send_start_positions(self):
        """Send start positions to trajectory planner

        Notes:
            - If CF is landed, starting position is considered 0.5m above
            - Doesn't send positon of extra CF if it's landed
        """
        # Send each CF starting position to planner
        start_positions = {}
        for cf_id, cf_vals in self.crazyflies.items():
            cf_pose = cf_vals.pose.pose

            # If landed, starting position is 0.5m above
            if cf_id in self.landed_cf_ids and cf_id not in self.extra_cf_list:
                start_positions[cf_id] = [cf_pose.position.x,
                                          cf_pose.position.y,
                                          cf_pose.position.z + TAKE_OFF_HEIGHT,
                                          yaw_from_quat(cf_pose.orientation)]

            else:
                start_positions[cf_id] = [cf_pose.position.x,
                                          cf_pose.position.y,
                                          cf_pose.position.z,
                                          yaw_from_quat(cf_pose.orientation)]

        self.traj_services["set_planner_positions"](position_type="start_position",
                                                    positions=str(start_positions))

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
                cf_initial_pose = cf_vals.initial_pose
                goals[cf_id] = [cf_initial_pose.position.x,
                                cf_initial_pose.position.y,
                                cf_initial_pose.position.z + TAKE_OFF_HEIGHT,
                                yaw_from_quat(cf_initial_pose.orientation)]

            # Goal of CF in formation
            elif cf_id not in self.extra_cf_list: # If CF in formation
                formation_goal = cf_vals.goals["formation"]
                goals[cf_id] = [formation_goal.x,
                                formation_goal.y,
                                formation_goal.z,
                                formation_goal.yaw]

                # Change CF positions
                #TODO Trade spots
                # if cf_id == 'cf1':
                #     cf_goal = self.crazyflies['cf2']['initial_pose']
                #     goals[cf_id] = [cf_goal.position.x,
                #                     cf_goal.position.y,
                #                     cf_goal.position.z + TAKE_OFF_HEIGHT,
                #                     yaw_from_quat(cf_goal.orientation)]
                # else:
                #     cf_goal = self.crazyflies['cf1']['initial_pose']
                #     goals[cf_id] = [cf_goal.position.x,
                #                     cf_goal.position.y,
                #                     cf_goal.position.z + TAKE_OFF_HEIGHT,
                #                     yaw_from_quat(cf_goal.orientation)]

            # If CF in extra and not landed, go to initial position
            elif cf_id not in self.landed_cf_ids:
                cf_initial_pose = cf_vals.initial_pose
                goals[cf_id] = [cf_initial_pose.position.x,
                                cf_initial_pose.position.y,
                                cf_initial_pose.position.z + 0.5,
                                yaw_from_quat(cf_initial_pose.orientation),]

        self.traj_services["set_planner_positions"](position_type="goal", positions=str(goals))

    def wait_for_take_off(self):
        """Wait that all CF of formation are in hover state
        """
        all_cf_in_air = False #: bool: True if all CFs are done taking off
        while not all_cf_in_air:
            all_cf_in_air = True

            if rospy.is_shutdown():
                break

            for cf_id, each_cf in self.crazyflies.items():
                if each_cf.state != "hover" and cf_id not in self.extra_cf_list:
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
                each_cf.update_goal()

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
                            (cf_vals.state not in ["landed", "land", "stop"] and\
                            cf_id in self.extra_cf_list):

                cf_initial_pose = cf_vals.initial_pose

                cf_vals.goals["goal"].x = cf_initial_pose.position.x
                cf_vals.goals["goal"].y = cf_initial_pose.position.y
                cf_vals.goals["goal"].z = cf_initial_pose.position.z
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
            traj_goal = each_cf.goals["trajectory"]
            # Make sure first msg was sent
            if traj_goal is not None:
                #If CF in extra and landed, don't follow goal
                if each_cf.state in ["landed", "land", "stop"] and cf_id in self.extra_cf_list:
                    each_cf.update_goal()
                else:
                    each_cf.update_goal("trajectory")

    def swarm_in_formation(self):
        """Swarm is in a specific formation. Lands extra CFs

        Formation can be moved /w the joystick
        """
        # Publish goal velocity
        self.formation_goal_vel = self.joy_swarm_vel
        self._pub_formation_goal_vel(self.formation_goal_vel)

        # Set CF goal to formation goal
        for cf_id, each_cf in self.crazyflies.items():
            # If CF part of formation, set goal to formation_goal
            if cf_id not in self.extra_cf_list:
                each_cf.update_goal("formation")

            # IF CF is not landed
            elif each_cf.state not in ["landed", "land", "stop"]:
                self.update_cf_goal_to_land()
                self._call_all_cf_service("land", cf_list=self.extra_cf_list)

            # If CF is already landed
            else:
                each_cf.update_goal()

    def hover_swarm(self):
        """CFs hover in place
        """
        # Set CF goal to current pose
        for _, each_cf in self.crazyflies.items():
            each_cf.update_goal()

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

        for cf_id, each_cf in self.crazyflies.items():

            cf_position = each_cf.pose.pose
            cf_position = np.array([cf_position.position.x,
                                    cf_position.position.y,
                                    cf_position.position.z])

            cf_goal = each_cf.goals["goal"]
            cf_goal = np.array([cf_goal.x,
                                cf_goal.y,
                                cf_goal.z])

            for other_id, other_cf in self.crazyflies.items():
                if other_id != cf_id:
                    other_pos = other_cf.pose.pose
                    other_pos = np.array([other_pos.position.x,
                                          other_pos.position.y,
                                          other_pos.position.z])

                    other_goal = other_cf.goals["goal"]
                    other_goal = np.array([other_goal.x,
                                           other_goal.y,
                                           other_goal.z])

                    p_dist = norm(dot(scaling_matrix_inv, cf_position - other_pos))
                    g_dist = norm(dot(scaling_matrix_inv, cf_goal - other_goal))

                    position_dist.append(p_dist)
                    goal_dist.append(g_dist)

        min_pos_dist = min(position_dist)
        min_goal_dist = min(goal_dist)

        # print "Min distances:"
        # print "\t position: %.2f" % min_pos_dist
        # print "\t goal: %.2f" % min_goal_dist

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

        if len(self.cf_list) > 1:
            self.check_positions()

        # Publish goal of each CF
        self._pub_cf_goals()

        self.rate.sleep()

class CrazyfliePy(object):
    """ Link between the swarm manager and a crazyflie.

    Controls one robot.
    """
    # Initialisation
    def __init__(self, cf_id):
        self.cf_id = cf_id

        # Subscribed services
        self._services = {"emergency": None,
                          "take_off": None,
                          "hover": None,
                          "land": None,
                          "stop": None,
                          "toggle_teleop": None,
                          "set_param": None}

        self.pose = None
        self.initial_pose = None

        self.goals = {"goal": Position(),
                      "formation": Position(),
                      "trajectory": Position()}

        self._goal_publisher = None

        self.state = ""

        self._init_cf()

    def _init_cf(self):
        """Initialize each CF

        Initialize services, publisher and subscriber
        """
        # Subscribe to services
        rospy.loginfo("Swarm: waiting services of %s " % self.cf_id)
        self._link_service("take_off", Empty)
        self._link_service("land", Empty)
        self._link_service("hover", Empty)
        self._link_service("stop", Empty)
        self._link_service("toggle_teleop", Empty)
        self._link_service("emergency", Empty)
        self._link_service("set_param", SetParam)
        rospy.loginfo("Swarm: found services of %s " % self.cf_id)

        # CF pose
        rospy.Subscriber("/%s/pose" % self.cf_id, PoseStamped, self._pose_handler)

        # Wait to receive first position
        while self.pose is None:
            pass

        # CF goal
        self._goal_publisher = rospy.Publisher('/' + self.cf_id + '/goal', Position, queue_size=1)

        self.goals["goal"].x = self.pose.pose.position.x
        self.goals["goal"].y = self.pose.pose.position.y
        self.goals["goal"].z = self.pose.pose.position.z

        rospy.Subscriber("/%s/formation_goal" % self.cf_id, Position, self._goal_handler, 0)
        rospy.Subscriber("/%s/trajectory_goal" % self.cf_id, Position, self._goal_handler, 1)

        rospy.Subscriber("/%s/state" % self.cf_id, String, self._state_handler)

    def _link_service(self, service_name, service_type):
        """Link service

        Args:
            service_name (str): Name of the serviec
            service_type (_): Type of the service
        """
        rospy.wait_for_service('/%s/%s' % (self.cf_id, service_name))
        self._services[service_name] = rospy.ServiceProxy('/%s/%s' % (self.cf_id, service_name),
                                                          service_type)

    # Topic Handlers
    def _pose_handler(self, pose_stamped):
        """Update current position of a cf

        Args:
            pose_stamped (PoseStamped): New pose of CF
        """
        self.pose = pose_stamped

    def _goal_handler(self, goal, goal_type):
        """Update formation or trajectory goal of CF.

        Args:
            goal (Position): Goal
            goal_type (int): Goal type: 0 -> formation_goal, 1 -> trajectory_goal
        """
        if goal_type == 0:
            self.goals["formation"] = goal

        elif goal_type == 1:
            self.goals["trajectory"] = goal

        else:
            rospy.logerr("Invalid goal type (%i) for CF" % goal_type)

    def _state_handler(self, state):
        self.state = state.data

    # Publisher
    def publish_goal(self):
        """Publish current CF goal
        """
        self.goals["goal"].header.seq += 1
        self.goals["goal"].header.stamp = rospy.Time.now()
        self._goal_publisher.publish(self.goals["goal"])

    def call_srv(self, srv_name, args=None, kwargs=None):
        """Call a CF service

        Args:
            srv_name (str): Name of srv to call
            args (list, optional): Service args. Defaults to None.
            kwargs (dict, optional): Service kwargs. Defaults to None.
        """
        if args is None:
            args = []

        if kwargs is None:
            kwargs = {}

        self._services[srv_name](*args, **kwargs)

    def update_initial_pose(self):
        """Update initial position to current pose
        """
        self.initial_pose = self.pose.pose

    def update_goal(self, goal_name=""):
        """Update current goal to one of other CF goal (trajectory, formation or current goal)

        Notes:
        "" or "goal": Set to current goal
        "traj" or "trajectory": Set to goal to trajectory goal
        "formation: Set goal to formation goal

        Args:
            goal_name (str, optional): To specify goal to set. Defaults to "".
        """
        new_goal = None
        if goal_name == "" or goal_name == "goal":
            new_goal = self.goals["goal"]
        elif goal_name == "traj" or goal_name == "trajectory":
            new_goal = self.goals["trajectory"]
        elif goal_name == "formation":
            new_goal = self.goals["formation"]

        self.goals["goal"] = new_goal

def generate_cf_list(n_cf):
    """Generate a list with the names of CFs

    Names are 'cf_id' i.e: cf_0, cf_1...

    Args:
        n_cf (int): Number of CF in the swarm

    Returns:
        list: List of CFs names
    """
    return [('cf_' + str(i)) for i in range(n_cf)]

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarm_controller', anonymous=False)

    # Get params
    CF_LIST = generate_cf_list(rospy.get_param("swarm")["n_cf"])
    rospy.set_param("cf_list", CF_LIST)

    TO_SIM = rospy.get_param("~to_sim", "False")

    TAKE_OFF_HEIGHT = rospy.get_param("swarm")["take_off_height"]
    GND_HEIGHT = rospy.get_param("swarm")["gnd_height"]
    MIN_CF_DIST = rospy.get_param("swarm")["min_dist"]
    MIN_GOAL_DIST = rospy.get_param("swarm")["min_goal_dist"]

    # Launch all CFs nodes
    launch_swarm(CF_LIST)

    # Initialize swarm
    SWARM = SwarmController(CF_LIST, TO_SIM)

    while not rospy.is_shutdown():
        SWARM.control_swarm()
