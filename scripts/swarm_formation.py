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

from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Twist
from std_srvs.srv import Empty, EmptyResponse, SetBool
import numpy as np
import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from crazyflie_charles.srv import SetFormation, PoseSet
from crazyflie_driver.msg import Position

from math import sin, cos, pi, sqrt, atan

offset = [0, 0, 0.2]

class FormationManager:
    def __init__(self, cf_list, to_sim):
        self.cf_list = cf_list
        self.n_cf = len(cf_list) #: (int) Number of CF in the swarm
        self.pose_cnt = 0 #: (int) To know when compute pose

        self.to_sim = to_sim #: (bool) Simulation or not
        self.trajectory_mode = False #: (bool) In trajectory mode, goes to a specified setpoint, In speed mode, controls variation of position
                                     # In trajectory mode, follow /swarm_goal, in speed_mode, follows /swarm_goal_vel
        
        self.abs_ctrl_mode = False #: In abs ctrl mode, moves in world/ In rel ctrl mode, moves relative to yaw

        self.rate = rospy.Rate(100)

        self.offset = offset #: (list of float) Swarm center offset

        self.formations = {"solo": SoloFormation(self.offset), 
                           "square": SquareFormation(self.offset),} #: All possible formations
        self.formation = None #: (str) Current formation

        self.start_positions = [] #: (list of list of float) Starting position of the formation, independant of CFs
        
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
            self.crazyflies[cf_id] = {  "pose": PoseStamped(),    # msg
                                        "goal": Position(),       # msg  
                                        "swarm_id": 0,            # int, position in the swarm
                                        "set_pose": None,         # service  
                                        "goal_pub": None}         # publisher

            # Add goal publisher
            self.crazyflies[cf_id]["goal_pub"] = rospy.Publisher('/%s/goal' % cf_id, Position, queue_size=1)
            
            # Subscribe to pose topic
            rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self.pose_handler, cf_id)

            # Find set pose service
            if self.to_sim:
                rospy.loginfo("Formation: waiting for services of %s " %  cf_id)
                # rospy.loginfo("Formation: waiting for %s service of %s " % ("set_pose", cf_id))
                rospy.wait_for_service('/%s/%s' % (cf_id, "set_pose"))
                # rospy.loginfo("Formation: found %s service of %s" % ("set_pose", cf_id))     
                self.crazyflies[cf_id]["set_pose"] = rospy.ServiceProxy('/%s/set_pose' % cf_id, PoseSet)
                rospy.loginfo("Formation: found services of %s " %  cf_id)

        # Start services
        rospy.Service('/set_formation', SetFormation, self.set_formation)
        rospy.Service('/set_offset', Empty, self.set_offset)        # TODO
        rospy.Service('/toggle_ctrl_mode', Empty, self.toggle_ctrl_mode) 
        rospy.Service('/update_swarm_goal', Empty, self.update_swarm_goal)
        rospy.Service('/formation_inc_scale', Empty, self.formation_inc_scale)
        rospy.Service('/formation_dec_scale', Empty, self.formation_dec_scale)

    # Services and subscriptions
    def pose_handler(self, pose_stamped, cf_id):
        self.crazyflies[cf_id]["pose"] = pose_stamped
        self.pose_cnt += 1

        # Compute swarm pose once every time all poses are updated
        if self.pose_cnt % self.n_cf == 0 and self.formation is not None:
            self.get_swarm_pose()
            self.pose_cnt = 0

    def swarm_goal_handler(self, goal_position):
        self.swarm_goal = goal_position
        if self.trajectory_mode and self.formation is not None:
            self.formation.compute_cf_goals(self.crazyflies, self.swarm_goal)

    def swarm_goal_vel_handler(self, goal_vel):
        self.swarm_goal_vel = goal_vel

        # Moves relative to world
        if self.abs_ctrl_mode:
            self.swarm_goal.x += self.swarm_goal_vel.linear.x
            self.swarm_goal.y += self.swarm_goal_vel.linear.y
            self.swarm_goal.z += self.swarm_goal_vel.linear.z
            self.swarm_goal.yaw += self.swarm_goal_vel.angular.z

        # Moves relative to orientation. X axis in front, y axis on toward the left, z axis up
        else:
            Vx = self.swarm_goal_vel.linear.x
            Vy = self.swarm_goal_vel.linear.y
            theta = self.swarm_goal.yaw
            dX = Vx * cos(theta) + Vy * cos(theta + pi/2.0)
            dY = Vx * sin(theta) + Vy * sin(theta + pi/2.0)
            self.swarm_goal.x += dX
            self.swarm_goal.y += dY
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
        pass

    def toggle_ctrl_mode(self, mode):
        self.abs_ctrl_mode = not self.abs_ctrl_mode
        if self.abs_ctrl_mode:
            rospy.loginfo("Formation: Control mode set to absolute")
        else:
            rospy.loginfo("Formation: Control mode set to realtive")

        return {}
    
    def update_swarm_goal(self, req=None):
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
    
    def formation_inc_scale(self, req):
        self.formation.change_scale(True)
        return {}

    def formation_dec_scale(self, req):
        self.formation.change_scale(False)
        return {}

    # Formation initialization methods
    def init_formation(self):
        self.formation.set_n_cf(len(self.cf_list))
        self.formation.check_n()
        self.start_positions = self.formation.compute_start_positions()

        if self.to_sim:
            # Set initial positions of each CF
            for (_, cf_attrs), (swarm_id, position) in zip(self.crazyflies.items(), self.start_positions.items()):
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
        
        rospy.sleep(0.3)    

        # Set CF goal to current pose
        for _, cf_attrs in self.crazyflies.items():
            cur_position = cf_attrs["pose"].pose
            cf_attrs["goal"].x = cur_position.position.x
            cf_attrs["goal"].y = cur_position.position.y
            cf_attrs["goal"].z = cur_position.position.z
            cf_attrs["goal"].yaw = yaw_from_quat(cur_position.orientation)

        rospy.sleep(0.2)
        self.update_swarm_goal()
        
    def compute_initial_pose(self, srv_call):
        if self.formation is not None:
            self.formation.compute_initial_pose()
    
    # Control methods
    def get_swarm_pose(self):
        self.swarm_pose = self.formation.compute_swarm_pose(self.crazyflies)
        return EmptyResponse()

    # Publishers
    def publish_cf_goals(self):
        for _, cf_attrs in self.crazyflies.items():
            cf_attrs["goal_pub"].publish(cf_attrs["goal"])        

    def publish_swarm_pose(self):
        self.swarm_pose_publisher.publish(self.swarm_pose)

    def publish_swarm_goal(self):
        self.swarm_goal_publisher.publish(self.swarm_goal)

    def run_formation(self):
        while not rospy.is_shutdown():
            if self.formation is not None:
                self.publish_cf_goals()
                self.publish_swarm_pose()
                self.publish_swarm_goal()
            self.rate.sleep()

class FormationType(object):
    def __init__(self, n_cf_supported=[], offset=[0, 0, 0]):
        self.n_cf = 0 #: (int) Number of CF in the formation
        
        self.cf_goals = {} #: (dict of Position) Target Pose of all the CF

        self.n_cf_supported = n_cf_supported #: (list of int) Different number of CF possible for each formation
        
        self.initial_offset = Pose() #: (Pose): Offset of the center of the formation from 0,0,0
        self.initial_offset.position.x = offset[0]
        self.initial_offset.position.y = offset[1]
        self.initial_offset.position.z = offset[2]

        self.scale = 1.0 #: (float) scale of the formation
        self.min_scale = 0
        self.max_scale = 5

    # General methods, valid between formations
    def check_n(self):
        if self.n_cf in self.n_cf_supported and self.n_cf > 0:
            rospy.loginfo("Formation made of %i crazyflies" % self.n_cf)
        else:
            rospy.logerr("Unsuported number of CFs")

    def get_n_cf(self):
        return self.n_cf

    def set_offset(self, x , y, z):
        self.initial_offset.position.x = x
        self.initial_offset.position.y = y
        self.initial_offset.position.z = z

    def set_n_cf(self, n):
        self.n_cf = n
        self.check_n
    
    def change_scale(self, to_inc):
        if to_inc:
            self.scale += 0.5

        else:
            self.scale -= 0.5
            
        if self.scale < self.min_scale: self.scale = self.min_scale
        if self.scale > self.max_scale: self.scale = self.max_scale

        self.update_scale()

    # Methods depending on formation
    def compute_start_positions(self):
        pass

    def compute_cf_goals(self, crazyflies, swarm_goal):
        """Compute goal of each crazyflie based on target position of the swarm

        Args:
            crazyflies (dict): Information of each Crazyflie
            swarm_goal (Position): Goal of the swarm
        """
        for _, cf_attrs in crazyflies.items():
            cf_attrs["goal"].x = cf_attrs["pose"].pose.position.x + swarm_goal.x
            cf_attrs["goal"].y = cf_attrs["pose"].pose.position.x + swarm_goal.y
            cf_attrs["goal"].z = cf_attrs["pose"].pose.position.x + swarm_goal.z
            # cf_attrs["goal"].yaw = cf_attrs["pose"].pose.position.x + swarm_goal.x

    def compute_cf_goals_vel(self, crazyflies, swarm_goal_vel):
        """Compute goal of each crazyflie based on target velocity of the swarm

        Args:
            crazyflies (dict): Information of each Crazyflie
            swarm_goal_vel (Twist): Goal of the swarm
        """
        for _, cf_attrs in crazyflies.items():
            cf_attrs["goal"].x += swarm_goal_vel.linear.x
            cf_attrs["goal"].y += swarm_goal_vel.linear.y
            cf_attrs["goal"].z += swarm_goal_vel.linear.z
            cf_attrs["goal"].yaw += swarm_goal_vel.angular.z

    def compute_swarm_pose(self, crazyflie_list):
        """Compute pose of the swarm

        Args:
            crazyflie_list (dict of dict): Attrs of each CF
        
        Returns:
            Pose: Swarm Pose
        """

        # To simplify, swarm pose is the average of all the poses
        swarm_pose = Pose()

        x = []
        y = []
        z = []
        yaw = []

        for _, cf_attrs in crazyflie_list.items(): 
            pose = cf_attrs["pose"].pose
            x.append(pose.position.x)
            y.append(pose.position.y)
            z.append(pose.position.z)
            yaw.append(yaw_from_quat(pose.orientation))

        swarm_pose.position.x = np.mean(x)
        swarm_pose.position.y = np.mean(y)
        swarm_pose.position.z = np.mean(z)
        [x, y, z, w] = quaternion_from_euler(0, 0, np.mean(yaw))
        swarm_pose.orientation.x = x
        swarm_pose.orientation.y = y
        swarm_pose.orientation.z = z
        swarm_pose.orientation.w = w


        
        return swarm_pose
    
    def update_scale(self):
        pass

class SquareFormation(FormationType):
    """Square formation

    Args:
        FormationType ([type]): [description]

    Notes:
        n_cf supported: 4 or 9
        scale: Length of a side

    Layouts:

        y
        |
        |
        |_____x

        1   3

        0   2

        ------

        2   5   8

        1   4   7

        0   3   6   

    """
    def __init__(self, offset=[0, 0, 0]):
        n_cf_supported = []
        super(SquareFormation, self).__init__(n_cf_supported, offset=offset)
        
        self.min_scale = 0.5

        # Attrs specific to square
        self.cf_per_side = 0 #: (float) Number of CF per side
        self.dist = 0 #: (float) Space between CFs
        self.center_dist = {} #: (dict of float) Keys: swarm id, Item: Distance from center
        self.angle = {} #: (dict of float) Keys: swarm id, Item: Angle(rad) from x axis 

    def check_n(self):
        # Check if n is a perfect square
        n = sqrt(self.n_cf)

        if n - int(n) == 0 and self.n_cf > 0:
            rospy.loginfo("Formation made of %i crazyflies" % self.n_cf)
        else:
            rospy.logerr("Unsuported number of CFs")

    # Setter
    def set_n_cf(self, n):
        super(SquareFormation, self).set_n_cf(n)
        self.cf_per_side = int(sqrt(self.n_cf)) # Number of CF per side
        self.dist = self.scale/(self.cf_per_side-1) # Space between CFs
        
    # Computing
    def compute_start_positions(self):        
        cf_num = 0
        center_x = self.scale/2.0
        center_y = self.scale/2.0

        for i in range(self.cf_per_side):
            for j  in range(self.cf_per_side):
                start_goal = Position()
                start_goal.x = i*self.dist
                start_goal.y = j*self.dist
                start_goal.z = 0
                start_goal.yaw = 0
                self.cf_goals[cf_num] = start_goal

                dX = start_goal.x - center_x
                dY = start_goal.y - center_y
                self.center_dist[cf_num] = sqrt(dX**2 + dY**2)

                if dX != 0:
                    theta = atan(dY/dX)
                    if dX < 0 and dY < 0: 
                        theta = theta - pi
                    elif dX < 0: 
                        theta = theta + pi

                else:
                    if dY > 0: theta = pi/2
                    else: theta = -pi/2

                self.angle[cf_num] = theta

                cf_num += 1
        
        return self.cf_goals

    def compute_cf_goals(self, crazyflies, swarm_goal):
        """Compute goal of each crazyflie based on target position of the swarm

        Args:
            crazyflies (dict): Information of each Crazyflie
            swarm_goal (Position): Goal of the swarm
        """
        # Compute position of all the CF

        for swarm_id in range(self.n_cf):
            yaw = swarm_goal.yaw
            theta = self.angle[swarm_id] + yaw
            dX = cos(theta) * self.center_dist[swarm_id]
            dY = sin(theta) * self.center_dist[swarm_id]

            self.cf_goals[swarm_id].x = swarm_goal.x + dX
            self.cf_goals[swarm_id].y = swarm_goal.y + dY
            self.cf_goals[swarm_id].z = swarm_goal.z
            self.cf_goals[swarm_id].yaw = yaw

        # Update crazyflies based on swarm ID
        for _, cf_attrs in crazyflies.items():
            id = cf_attrs["swarm_id"]
            cf_attrs["goal"].x = self.cf_goals[id].x
            cf_attrs["goal"].y = self.cf_goals[id].y
            cf_attrs["goal"].z = self.cf_goals[id].z
            cf_attrs["goal"].yaw = self.cf_goals[id].yaw
    
    def update_scale(self):
        self.dist = self.scale/(self.cf_per_side-1) # Space between CFs
        self.compute_start_positions()

class SoloFormation(FormationType):
    def __init__(self, offset=[0, 0, 0]):
        n_cf_supported = [1]
        super(SoloFormation, self).__init__(n_cf_supported, offset=offset)

    def compute_start_positions(self):
        return [self.initial_offset.position.x,  self.initial_offset.position.y, self.initial_offset.position.z]
        
        # for _, each_cf in self.cf_goals.items():
        #     each_cf.position.x = self.initial_offset.position.x 
        #     each_cf.position.y = self.initial_offset.position.y
        #     each_cf.position.z = self.initial_offset.position.z

def calculate_rot(start_orientation, rot):
    """Apply rotation to quaternion

    Args:
        start_orientation (Quaternion): Initial orientation
        rot (Vector3): Angular speed to apply

    Returns:
        Quaternion: Result
    """
    rot_q = quaternion_from_euler(rot.x, rot.y, rot.z)
    orig_q = [start_orientation.x, 
                start_orientation.y, 
                start_orientation.z, 
                start_orientation.w]
    res_q = quaternion_multiply(rot_q, orig_q)

    res_msg = Quaternion()
    res_msg.x = res_q[0]
    res_msg.y = res_q[1]
    res_msg.z = res_q[2]
    res_msg.w = res_q[3]

    return res_msg

def yaw_from_quat(quaternion):
    _, _, yaw = euler_from_quaternion(  [quaternion.x,
                                        quaternion.y,
                                        quaternion.z,
                                        quaternion.w])
    return yaw

def quat_from_yaw(yaw):
    x, y, z, w = quaternion_from_euler(0, 0 , yaw)
    return Quaternion(x, y, z, w)

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarm_formation_manager', anonymous=False)
    rospy.loginfo('Initialization of swarm formations manager')

    # Get params
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    to_sim = rospy.get_param("~to_sim", "False")
    
    # Initialize swarm
    formation_manger = FormationManager(cf_list, to_sim)

    formation_manger.run_formation()

    rospy.spin()




    

    