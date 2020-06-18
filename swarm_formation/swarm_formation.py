#!/usr/bin/env python

"""To manage the formation of the swarm

In charge of computing the positions of all the crazyflies.

Avalaible formations:
    - Square
    - Pyramid
    - Circle
    - V
    - Ligne

Services:
    - set_formation: Set formation type
    - set_offset: Set offset of goal
    - toggle_ctrl_mode: Toggle between absolute and relative ctrl mode
    - update_swarm_goal: Compute current formation center
    - formation_inc_scale: Increase scale of formation
    - formation_dec_scale: Decrease scale of formation
    - get_formation_list: Returns a list /w all possible formations

Subscribed Services:
    None

Subscribtion:
    /formation_goal_vel: Velocity of formation center
    /cfx/pose: To compute formation current center

Publisher:
    /formation_center: Center of the formation
    /formation_goal: Goal of formation
    /cfx/formation_goal: Goal, in formation, of each CF
"""
from math import sin, cos, pi
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_srvs.srv import Empty, EmptyResponse
import rospy
from crazyflie_charles.srv import SetFormation, GetFormationList
from crazyflie_driver.msg import Position

from general_formation import yaw_from_quat
from square_formation import SquareFormation
from line_formation import LineFormation
from circle_formation import CircleFormation
from pyramid_formation import PyramidFormation
from v_formation import VFormation

FORMATION_INITIAL_POS = Pose()
FORMATION_INITIAL_POS.position.x = 0.5
FORMATION_INITIAL_POS.position.y = 0.5
FORMATION_INITIAL_POS.position.z = 0.5
FORMATION_INITIAL_POS.orientation.w = 1.0

class FormationManager(object):
    """To manage to position of all CF in the formation.

    Associates formation position /w a CF

    """
    def __init__(self, cf_list, to_sim):
        self.cf_list = cf_list
        self.n_cf = len(cf_list) #: (int) Number of CF in the swarm
        self.pose_cnt = 0 #: (int) To know when compute pose

        self.to_sim = to_sim #: (bool) Simulation or not

        #: (bool) In trajectory mode, goes to a specified setpoint.
        #  In speed mode, controls variation of position
        #  In trajectory mode, follows /swarm_goal, in speed_mode, follows /swarm_goal_vel
        self.trajectory_mode = False

        #: In abs ctrl mode, moves in world/ In rel ctrl mode, moves relative to yaw
        self.abs_ctrl_mode = False

        self.rate = rospy.Rate(100)

        self.initial_formation_pos = FORMATION_INITIAL_POS #: Position: formation start position

        #: All possible formations
        self.formations = {"square": SquareFormation(self.initial_formation_pos),
                           "v": VFormation(self.initial_formation_pos),
                           "pyramid": PyramidFormation(self.initial_formation_pos),
                           "circle": CircleFormation(self.initial_formation_pos),
                           "line": LineFormation(self.initial_formation_pos),}
        self.formation = None #: (str) Current formation

        #: (list of list of float): Starting pos of each agent in formation, independant of CF id
        self.start_positions = []

        self.crazyflies = {} #: (dict of list) Information of each CF

        # Publisher
        self.formation_pose_pub = rospy.Publisher('/formation_pose', Pose, queue_size=1)
        self.formation_goal_pub = rospy.Publisher('/formation_goal', Position, queue_size=1)
        self.formation_pose = Pose()
        self.formation_goal = Position()
        self.formation_goal_vel = Twist()

        if self.initial_formation_pos is not None:
            self.formation.pose = self.initial_formation_pos

        # Subscribers
        rospy.Subscriber("/formation_goal_vel", Twist, self.formation_goal_vel_handler)

        # Initialize each CF
        for cf_id in cf_list:
            self.crazyflies[cf_id] = {"pose": PoseStamped(),        # msg
                                      "formation_goal": Position(), # msg
                                      "swarm_id": 0,                # int, id in the swarm
                                      "formation_goal_pub": None}   # publisher

            # Add goal publisher
            self.crazyflies[cf_id]["formation_goal_pub"] =\
                rospy.Publisher('/%s/formation_goal' % cf_id, Position, queue_size=1)

            # Subscribe to pose topic
            rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self.pose_handler, cf_id)

        # Start services
        rospy.Service('/set_formation', SetFormation, self.set_formation)
        rospy.Service('/set_offset', Empty, self.set_offset) # TODO: Set offset
        rospy.Service('/toggle_ctrl_mode', Empty, self.toggle_ctrl_mode)
        rospy.Service('/update_swarm_goal', Empty, self.update_formation_goal)
        rospy.Service('/formation_inc_scale', Empty, self.formation_inc_scale)
        rospy.Service('/formation_dec_scale', Empty, self.formation_dec_scale)
        rospy.Service('/get_formations_list', GetFormationList, self.return_formation_list)

    # Services and subscriptions
    def pose_handler(self, pose_stamped, cf_id):
        """Update current position of a cf

        Args:
            pose_stamped (PoseStamped): New pose of CF
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id]["pose"] = pose_stamped
        self.pose_cnt += 1

        # Compute formation center once every time all cf poses are updated
        if self.pose_cnt % self.n_cf == 0 and self.formation is not None:
            self.get_swarm_pose()
            self.pose_cnt = 0

    def formation_goal_vel_handler(self, goal_vel):
        """To change formation goal based on a velocity

        Depends on ctrl mode

        Args:
            goal_vel (Twist): Swarm goal velocity
        """
        self.formation_goal_vel = goal_vel

        # Moves relative to world
        if self.abs_ctrl_mode:
            self.formation_goal.x += self.formation_goal_vel.linear.x
            self.formation_goal.y += self.formation_goal_vel.linear.y
            self.formation_goal.z += self.formation_goal_vel.linear.z
            self.formation_goal.yaw += self.formation_goal_vel.angular.z

        # Moves relative to orientation. X axis in front, y axis on toward the left, z axis up
        else:
            x_vel = self.formation_goal_vel.linear.x
            y_vel = self.formation_goal_vel.linear.y
            theta = self.formation_goal.yaw
            x_dist = x_vel * cos(theta) + y_vel * cos(theta + pi/2.0)
            y_dist = x_vel * sin(theta) + y_vel * sin(theta + pi/2.0)
            self.formation_goal.x += x_dist
            self.formation_goal.y += y_dist
            self.formation_goal.z += self.formation_goal_vel.linear.z
            self.formation_goal.yaw += self.formation_goal_vel.angular.z

        # If in velocity ctrl
        if not self.trajectory_mode and self.formation is not None:
            # self.formation.compute_cf_goals_vel(self.crazyflies, self.swarm_goal_vel)
            self.formation.compute_cf_goals(self.crazyflies, self.formation_goal)

    def set_formation(self, srv_call):
        """Set formation

        Args:
            srv_call (SetFormation): Formation to set

        Returns:
            bool: success
        """
        form_to_set = srv_call.formation
        valid_formation = True

        if form_to_set in self.formations.keys():
            rospy.loginfo("Formation: Setting formation to %s" % form_to_set)
            self.formation = self.formations[form_to_set]
            self.init_formation()

        else:
            rospy.logerr("Formation: Invalid formation: %s" % form_to_set)
            valid_formation = False

        return valid_formation

    def set_offset(self, srv_call):
        """Set offset of the swarm

        Not implemented

        Args:
            srv_call ([type]): [description]
        """
        pass

    def toggle_ctrl_mode(self, _):
        """Toggle control mode

        Absolute mode: Controls in world reference

        Relative mode: Controls depending of swarm orientation
        """
        self.abs_ctrl_mode = not self.abs_ctrl_mode
        if self.abs_ctrl_mode:
            rospy.loginfo("Formation: Control mode set to absolute")
        else:
            rospy.loginfo("Formation: Control mode set to relative")

        return {}

    def update_formation_goal(self, _=None):
        """Update formation goal to match current position

        - formation_goal is set to formation_pose
        - cf formation_goal set to cf current_pose

        Args:
            req (Empty, optional): service. Defaults to None.

        Returns:
            EmprtyResponse

        Notes:
            Call from service and in initialisation
        """
        #TODO: REMOVE, Update, formation goal cant move if not following formation
        # Update swarm position
        self.get_swarm_pose()

        # Update swarm goal to match current position
        self.formation_goal.x = self.formation_pose.position.x
        self.formation_goal.y = self.formation_pose.position.y
        self.formation_goal.z = self.formation_pose.position.z
        self.formation_goal.yaw = yaw_from_quat(self.formation_pose.orientation)

        # Update CF goals to match current position
        for _, cf_attrs in self.crazyflies.items():
            cur_position = cf_attrs["pose"].pose
            cf_attrs["formation_goal"].x = cur_position.position.x
            cf_attrs["formation_goal"].y = cur_position.position.y
            cf_attrs["formation_goal"].z = cur_position.position.z
            cf_attrs["formation_goal"].yaw = yaw_from_quat(cur_position.orientation)

        return EmptyResponse()

    def formation_inc_scale(self, _):
        """Service to increase scale of the formation
        """
        self.formation.change_scale(True)
        return {}

    def formation_dec_scale(self, _):
        """Service to reduce scale of the formation
        """
        self.formation.change_scale(False)
        return {}

    def return_formation_list(self, _):
        """To get a list of all possible formations

        Returns:
            list of str: Possible formation
        """
        possible_formations = self.formations.keys()
        return {"formations": ','.join(possible_formations)}

    # Formation initialization methods
    def init_formation(self):
        """Initialize formation goal and cf positions
        """
        self.formation.set_n_cf(len(self.cf_list))
        self.start_positions = self.formation.compute_start_positions()

        # Set CF goal and swarm_id
        for (_, cf_attrs), (swarm_id, position) in zip(self.crazyflies.items(),
                                                       self.start_positions.items()):
            cf_attrs["formation_goal"] = position
            cf_attrs["swarm_id"] = swarm_id

        rospy.sleep(0.2)
        # self.update_formation_goal()

    # Control methods
    def get_swarm_pose(self):
        """Update current position of the swarm

        Empty service
        """
        self.formation_pose = self.formation.compute_swarm_pose(self.crazyflies)
        return EmptyResponse()

    # Publishers
    def publish_cf_formation_goals(self):
        """Publish formation goal of each CF
        """
        for _, cf_attrs in self.crazyflies.items():
            if rospy.is_shutdown():
                break
            cf_attrs["formation_goal_pub"].publish(cf_attrs["formation_goal"])

    def publish_formation_pose(self):
        """Publish current position of the formation (center)
        """
        self.formation_pose_pub.publish(self.formation_pose)

    def publish_formation_goal(self):
        """Publish current goal of the formation
        """
        self.formation_goal_pub.publish(self.formation_goal)

    def run_formation(self):
        """Execute formation
        """
        while not rospy.is_shutdown():
            if self.formation is not None:
                self.publish_cf_formation_goals()
                self.publish_formation_pose()
                self.publish_formation_goal()
            self.rate.sleep()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarm_formation_manager', anonymous=False)
    rospy.loginfo('Initialization of swarm formations manager')

    # Get params
    CF_LIST = rospy.get_param("~cf_list", "['cf1']")
    TO_SIM = rospy.get_param("~to_sim", "False")

    # Initialize swarm
    FORMATION_MANAGER = FormationManager(CF_LIST, TO_SIM)

    FORMATION_MANAGER.run_formation()

    rospy.spin()
