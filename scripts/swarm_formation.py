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
"""

from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Twist
from std_srvs.srv import Empty, SetBool
import numpy as np
import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from crazyflie_charles.srv import SetFormation, PoseSet
from crazyflie_driver.msg import Position

offset = [0, 0, 0.2]

class FormationManager:
    def __init__(self, cf_list, to_sim):
        self.cf_list = cf_list
        self.n_cf = len(cf_list) #: (int) Number of CF in the swarm
        self.pose_cnt = 0 #: (int) To know when compute pose

        self.to_sim = to_sim
        self.offset = offset
        self.rate = rospy.Rate(100)

        self.formations = {"solo": SoloFormation(self.cf_list, self.offset), 
                           "square": SquareFormation(self.cf_list, self.offset),} #: All possible formations
        self.formation = None
        self.start_positions = []
                
        self.swarm_pose_publisher = rospy.Publisher('/swarm_pose', Pose, queue_size=1)
        self.swarm_goal_publisher = rospy.Publisher('/swarm_goal', Position, queue_size=1)
        self.swarm_pose = Pose()
        self.swarm_goal = Position()
        self.swarm_goal_vel = Twist()
        rospy.Subscriber("/swarm_goal", Position, self.swarm_goal_handler)
        rospy.Subscriber("/swarm_goal_vel", Twist, self.swarm_goal_vel_handler)

        self.crazyflies = {} #: (dict of list) Information of each CF

        self.trajectory_mode = False #: In trajectory mode, goes to a specified setpoint, In speed mode, controls variation of position
    

        # Initialize each CF
        for cf_id in cf_list:
            self.crazyflies[cf_id] = {  "pose": PoseStamped(),    # msg
                                        "goal": Position(),       # msg  
                                        "set_pose": None,             # service  
                                        "goal_pub": None}             # publisher

            # Add goal publisher
            self.crazyflies[cf_id]["goal_pub"] = rospy.Publisher('/%s/goal' % cf_id, Position, queue_size=1)
            
            # Subscribe to pose topic
            rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self.pose_handler, cf_id)

            # Find set pose service
            if self.to_sim:
                rospy.loginfo("Formation: waiting for %s service of %s " % ("set_pose", cf_id))
                rospy.wait_for_service('/%s/%s' % (cf_id, "set_pose"))
                rospy.loginfo("Formation: found %s service of %s" % ("set_pose", cf_id))     
                self.crazyflies[cf_id]["set_pose"] = rospy.ServiceProxy('/%s/set_pose' % cf_id, PoseSet)

        # Start services
        rospy.Service('/set_formation', SetFormation, self.set_formation)
        rospy.Service('/set_offset', SetFormation, self.set_offset)        # TODO
        rospy.Service('/set_ctrl_mode', SetFormation, self.set_ctrl_mode)        # TODO
        rospy.Service('/compute_initial_pose', Empty, self.compute_initial_pose) # Not used

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

    def init_formation(self):
        self.formation.check_n()
        self.start_positions = self.formation.compute_start_positions()

        if self.to_sim:
            # Set initial positions of each CF
            for (_, cf_attrs), positions in zip(self.crazyflies.items(), self.start_positions):
                pose = Pose()
                pose.position.x = positions[0]
                pose.position.y = positions[1]                
                pose.position.z = positions[2]                
                pose.orientation.w = 1

                cf_attrs["set_pose"](pose)

        else:
            #TODO: Pos initial exp
            pass
        
        rospy.sleep(0.2)    
        # Set CF goal to current pose
        for _, cf_attrs in self.crazyflies.items():
            cur_position = cf_attrs["pose"].pose
            cf_attrs["goal"].x = cur_position.position.x
            cf_attrs["goal"].y = cur_position.position.y
            cf_attrs["goal"].z = cur_position.position.z
            cf_attrs["goal"].yaw = yaw_from_quat(cur_position.orientation)

        rospy.sleep(0.2)
        self.get_swarm_pose()
        self.swarm_goal.x = self.swarm_pose.position.x 
        self.swarm_goal.y = self.swarm_pose.position.y
        self.swarm_goal.z = self.swarm_pose.position.z 
        self.swarm_goal.yaw = yaw_from_quat(self.swarm_pose.orientation)
        
    def set_offset(self, srv_call):
        pass
    
    def set_ctrl_mode(self, mode):
        pass

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
        self.swarm_goal.x += self.swarm_goal_vel.linear.x
        self.swarm_goal.y += self.swarm_goal_vel.linear.y
        self.swarm_goal.z += self.swarm_goal_vel.linear.z
        self.swarm_goal.yaw += self.swarm_goal_vel.angular.z

        if not self.trajectory_mode and self.formation is not None:
            self.formation.compute_cf_goals_vel(self.crazyflies, self.swarm_goal_vel)

    def compute_initial_pose(self, srv_call):
        if self.formation is not None:
            self.formation.compute_initial_pose()
    
    def get_swarm_pose(self):
        self.swarm_pose = self.formation.compute_swarm_pose(self.crazyflies)

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
    def __init__(self, cf_list, n_cf_supported=[], offset=[0, 0, 0]):
        self.cf_list = cf_list
        self.n_cf = len(cf_list) #: (int) Number of CF in the formation
        
        # self.cf_goals = {} #: (dict of Pose) Target Pose of all the CF
        
        # for each_cf in cf_list:
        #     p = Pose()
        #     p.orientation.w = 1
        #     self.cf_goals[each_cf] = p

        self.swarm_goal = Position()
        self.swarm_goal.yaw = 0

        self.n_cf_supported = n_cf_supported #: (list of int) Different number of CF possible for each formation
        self.initial_offset = Pose() #: (Pose): Offset of the center of the formation from 0,0,0
        self.initial_offset.position.x = offset[0]
        self.initial_offset.position.y = offset[1]
        self.initial_offset.position.z = offset[2]

    def check_n(self):
        if self.n_cf in self.n_cf_supported and self.n_cf > 0:
            rospy.loginfo("Formation made of %i crazyflies" % self.n_cf)
        else:
            rospy.logerr("Unsuported number of CFs")

    def get_n_cf(self):
        return self.n_cf

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

    def set_offset(self, x , y, z):
        self.initial_offset.position.x = x
        self.initial_offset.position.y = y
        self.initial_offset.position.z = z

class SquareFormation(FormationType):
    def __init__(self, cf_list, offset=[0, 0, 0]):
        n_cf_supported = [4, 9]
        super(SquareFormation, self).__init__(cf_list, n_cf_supported, offset=offset)

    def compute_start_positions(self):
        pose_list = []
        
        l = 1.0 # Lenght of the square

        k = int(np.sqrt(self.n_cf))
        l = l/(k-1)
        for i in range(k):
            for j  in range(k):
                pose_list.append([i*l, j*l, 0])

        return pose_list

        # for each_cf, position in zip(self.cf_goals.items(), pose_list):
        #     each_cf[1].position.x = position[0] + self.initial_offset.position.x 
        #     each_cf[1].position.y = position[1] + self.initial_offset.position.y
        #     each_cf[1].position.z = position[2] + self.initial_offset.position.z

class SoloFormation(FormationType):
    def __init__(self, cf_list, offset=[0, 0, 0]):
        n_cf_supported = [1]
        super(SoloFormation, self).__init__(cf_list, n_cf_supported, offset=offset)

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




    

    