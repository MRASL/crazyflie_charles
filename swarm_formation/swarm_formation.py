#!/usr/bin/env python

"""To manage the formation of the swarm

In charge of computing the positions of all the crazyflies.

Notes:
    Avalaible formations (# cf):
        - Square (4, 9)
        - Pyramid (4, 9)
        - Circle (X)
        - V (X)
        - Ligne (X)

TODO:
    *Add possibility to scale formations
    *Change between formations
    *Offset
"""
from math import sin, cos, pi
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_srvs.srv import Empty, EmptyResponse
import rospy
from crazyflie_charles.srv import SetFormation, PoseSet, GetFormationList
from crazyflie_driver.msg import Position

from general_formation import yaw_from_quat, quat_from_yaw
from square_formation import SquareFormation
from line_formation import LineFormation
from circle_formation import CircleFormation
from pyramid_formation import PyramidFormation
from v_formation import VFormation

OFFSET = [0, 0, 0.2]

class FormationManager(object):
    """To manage to position of all CF in the formation.
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

        self.offset = OFFSET #: (list of float) Swarm center offset

        self.formation = None #: (str) Current formation
        self.formations = {"square": SquareFormation(self.offset), #: All possible formations
                           "v": VFormation(self.offset),
                           "pyramid": PyramidFormation(self.offset),
                           "circle": CircleFormation(self.offset),
                           "line": LineFormation(self.offset),}

        #: (list of list of float) Starting position of the formation, independant of CFs
        self.start_positions = []

        self.crazyflies = {} #: (dict of list) Information of each CF


        # Publisher
        self.swarm_pose_publisher = rospy.Publisher('/swarm_pose', Pose, queue_size=1)
        self.swarm_goal_publisher = rospy.Publisher('/swarm_goal', Position, queue_size=1)
        self.swarm_pose = Pose()
        self.swarm_goal = Position()
        self.swarm_goal_vel = Twist()

        # Subscribers
        rospy.Subscriber("/swarm_goal", Position, self.swarm_goal_handler)
        rospy.Subscriber("/swarm_goal_vel", Twist, self.swarm_goal_vel_handler)

        # Initialize each CF
        for cf_id in cf_list:
            self.crazyflies[cf_id] = {"pose": PoseStamped(),    # msg
                                      "goal": Position(),       # msg
                                      "swarm_id": 0,            # int, position in the swarm
                                      "set_pose": None,         # service
                                      "goal_pub": None}         # publisher

            # Add goal publisher
            self.crazyflies[cf_id]["goal_pub"] = rospy.Publisher('/%s/goal' % cf_id,
                                                                 Position, queue_size=1)

            # Subscribe to pose topic
            rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self.pose_handler, cf_id)

            # Find set pose service
            if self.to_sim:
                rospy.loginfo("Formation: waiting for services of %s " %  cf_id)
                # rospy.loginfo("Formation: waiting for %s service of %s " % ("set_pose", cf_id))
                rospy.wait_for_service('/%s/%s' % (cf_id, "set_pose"))
                # rospy.loginfo("Formation: found %s service of %s" % ("set_pose", cf_id))
                self.crazyflies[cf_id]["set_pose"] = rospy.ServiceProxy('/%s/set_pose' % cf_id,
                                                                        PoseSet)
                rospy.loginfo("Formation: found services of %s " %  cf_id)

        # Start services
        rospy.Service('/set_formation', SetFormation, self.set_formation)
        rospy.Service('/set_offset', Empty, self.set_offset)        # TODO: Set offset
        rospy.Service('/toggle_ctrl_mode', Empty, self.toggle_ctrl_mode)
        rospy.Service('/update_swarm_goal', Empty, self.update_swarm_goal)
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

        # Compute swarm pose once every time all poses are updated
        if self.pose_cnt % self.n_cf == 0 and self.formation is not None:
            self.get_swarm_pose()
            self.pose_cnt = 0

    def swarm_goal_handler(self, goal_position):
        """Update swarm goal

        Args:
            goal_position (Positions): New swarm goal
        """
        self.swarm_goal = goal_position
        if self.trajectory_mode and self.formation is not None:
            self.formation.compute_cf_goals(self.crazyflies, self.swarm_goal)

    def swarm_goal_vel_handler(self, goal_vel):
        """To change swarm goal based on a velocity

        Depends on ctrl mode

        Args:
            goal_vel (Twist): Swarm goal velocity
        """
        self.swarm_goal_vel = goal_vel

        # Moves relative to world
        if self.abs_ctrl_mode:
            self.swarm_goal.x += self.swarm_goal_vel.linear.x
            self.swarm_goal.y += self.swarm_goal_vel.linear.y
            self.swarm_goal.z += self.swarm_goal_vel.linear.z
            self.swarm_goal.yaw += self.swarm_goal_vel.angular.z

        # Moves relative to orientation. X axis in front, y axis on toward the left, z axis up
        else:
            x_vel = self.swarm_goal_vel.linear.x
            y_vel = self.swarm_goal_vel.linear.y
            theta = self.swarm_goal.yaw
            x_dist = x_vel * cos(theta) + y_vel * cos(theta + pi/2.0)
            y_dist = x_vel * sin(theta) + y_vel * sin(theta + pi/2.0)
            self.swarm_goal.x += x_dist
            self.swarm_goal.y += y_dist
            self.swarm_goal.z += self.swarm_goal_vel.linear.z
            self.swarm_goal.yaw += self.swarm_goal_vel.angular.z

        # If in velocity ctrl
        if not self.trajectory_mode and self.formation is not None:
            # self.formation.compute_cf_goals_vel(self.crazyflies, self.swarm_goal_vel)
            self.formation.compute_cf_goals(self.crazyflies, self.swarm_goal)

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

    def update_swarm_goal(self, _=None):
        """Update swarm goal to match current position

        Args:
            req (Empty, optional): service. Defaults to None.

        Returns:
            EmprtyResponse

        Notes:
            Call from service and in initialisation
        """
        # Update swarm position
        self.get_swarm_pose()

        # Update swarm goal to match current position
        self.swarm_goal.x = self.swarm_pose.position.x
        self.swarm_goal.y = self.swarm_pose.position.y
        self.swarm_goal.z = self.swarm_pose.position.z
        self.swarm_goal.yaw = yaw_from_quat(self.swarm_pose.orientation)

        # Update CF goals to match current position
        for _, cf_attrs in self.crazyflies.items():
            cur_position = cf_attrs["pose"].pose
            cf_attrs["goal"].x = cur_position.position.x
            cf_attrs["goal"].y = cur_position.position.y
            cf_attrs["goal"].z = cur_position.position.z
            cf_attrs["goal"].yaw = yaw_from_quat(cur_position.orientation)

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

        if self.to_sim:
            # Set initial positions of each CF
            for (_, cf_attrs), (swarm_id, position) in zip(self.crazyflies.items(),
                                                           self.start_positions.items()):
                pose = Pose()
                pose.position.x = position.x
                pose.position.y = position.y
                pose.position.z = position.z
                pose.orientation = quat_from_yaw(position.yaw)

                cf_attrs["set_pose"](pose)
                cf_attrs["swarm_id"] = swarm_id

        else:
            #TODO: Pos initial exp
            pass

        rospy.sleep(0.5)

        # Set CF goal to current pose
        for _, cf_attrs in self.crazyflies.items():
            cur_position = cf_attrs["pose"].pose
            cf_attrs["goal"].x = cur_position.position.x
            cf_attrs["goal"].y = cur_position.position.y
            cf_attrs["goal"].z = cur_position.position.z
            cf_attrs["goal"].yaw = yaw_from_quat(cur_position.orientation)

        rospy.sleep(0.2)
        self.update_swarm_goal()

    # Control methods
    def get_swarm_pose(self):
        """Update current position of the swarm

        Empty service
        """
        self.swarm_pose = self.formation.compute_swarm_pose(self.crazyflies)
        return EmptyResponse()

    # Publishers
    def publish_cf_goals(self):
        """Publish swarm goal of each CF
        """
        for _, cf_attrs in self.crazyflies.items():
            if rospy.is_shutdown():
                break
            cf_attrs["goal_pub"].publish(cf_attrs["goal"])

    def publish_swarm_pose(self):
        """Publish current position of the swarm (center)
        """
        self.swarm_pose_publisher.publish(self.swarm_pose)

    def publish_swarm_goal(self):
        """Publish current goal of the swarm
        """
        self.swarm_goal_publisher.publish(self.swarm_goal)

    def run_formation(self):
        """Execute formation
        """
        while not rospy.is_shutdown():
            if self.formation is not None:
                self.publish_cf_goals()
                self.publish_swarm_pose()
                self.publish_swarm_goal()
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
