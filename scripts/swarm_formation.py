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

from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_srvs.srv import Empty
import numpy as np
import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply
from crazyflie_charles.srv import SetFormation
from crazyflie_driver.msg import Position

offset = [0, 0, 0.2]

class FormationManager:
    def __init__(self, cf_list, to_sim):
        self.cf_list = cf_list
        self.to_sim = to_sim
        self.offset = offset
        self.rate = rospy.Rate(100)

        self.formations = {"solo": SoloFormation(self.cf_list, self.offset), 
                           "square": SquareFormation(self.cf_list, self.offset),} #: All possible formations
        self.formation = None
                
        self.swarm_pose_publisher = rospy.Publisher('/swarm_pose', Pose, queue_size=1)

        self.goal_publisher = {}
        self.cf_poses = {}

        for cf_id in cf_list:
            self.goal_publisher[cf_id] = rospy.Publisher('/%s/goal' % cf_id, Position, queue_size=1)
            rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self.pose_handler, cf_id)

        # Start services
        rospy.Service('/set_formation', SetFormation, self.set_formation)
        rospy.Service('/set_offset', SetFormation, self.set_offset)        # TODO
        rospy.Service('/compute_initial_pose', Empty, self.compute_initial_pose)

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
            rospy.loginfo("Setting formation to %s" % form_to_set)
            self.formation = self.formations[form_to_set]
            self.formation.check_n()

        else:
            rospy.logerr("Invalid formation: %s" % form_to_set)
            valid_formation = False

        return valid_formation

    def set_offset(self, srv_call):
        pass
        
    def pose_handler(self, pose_stamped, cf_id):
        self.cf_poses[cf_id] = pose_stamped

    def compute_initial_pose(self, srv_call):
        if self.formation is not None:
            self.formation.compute_initial_pose()

    def publish_cf_goals(self):
        for cf_id, cf_goal in self.goal_publisher.items():
            cf_goal.publish(self.formation.cf_goals[cf_id])        

    def publish_swarm_pose(self):
        self.swarm_pose_publisher.publish(self.formation.swarm_pose)

    def run_formation(self):
        while not rospy.is_shutdown():
            if self.formation is not None:
                # self.publish_cf_goals()
                self.publish_swarm_pose()
            self.rate.sleep()


class FormationType(object):
    def __init__(self, cf_list, n_cf_supported=[], offset=[0, 0, 0]):
        self.cf_list = cf_list
        self.n_cf = len(cf_list) #: (int) Number of CF in the formation
        
        self.cf_goals = {} #: (dict of Pose) Target Pose of all the CF
        for each_cf in cf_list:
            p = Pose()
            p.orientation.w = 1
            self.cf_goals[each_cf] = p

        self.swarm_pose = Pose()

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

    def compute_initial_pose(self):
        pass
    
    def compute_new_pose(self):
        pass

    def get_swarm_pose(self):
        # To simplify, swarm pose is the average of all the poses

        x = []
        y = []
        z = []
        # yaw = []

        for _, cf in self.crazyflies.items(): 
            cf["initial_pose"] = cf["get_pose"]().pose
            cf["goal_msg"] = cf["initial_pose"]

            x.append(cf["initial_pose"].position.x)
            y.append(cf["initial_pose"].position.y)
            z.append(cf["initial_pose"].position.z)

        self.swarm_pose.position.x = np.mean(x)
        self.swarm_pose.position.y = np.mean(y)
        self.swarm_pose.position.z = np.mean(z)
        
        self.swarm_goal = self.swarm_pose
        self.formation.swarm_goal = self.swarm_goal

    def set_offset(self, x , y, z):
        self.initial_offset.position.x = x
        self.initial_offset.position.y = y
        self.initial_offset.position.z = z

class SquareFormation(FormationType):
    def __init__(self, cf_list, offset=[0, 0, 0]):
        n_cf_supported = [4, 9]
        super(SquareFormation, self).__init__(cf_list, n_cf_supported, offset=offset)

    def compute_initial_pose(self):
        pose_list = []
        
        l = 1.0 # Lenght of the square

        k = int(np.sqrt(self.n_cf))
        l = l/(k-1)
        for i in range(k):
            for j  in range(k):
                pose_list.append([i*l, j*l, 0])

        for each_cf, position in zip(self.cf_goals.items(), pose_list):
            each_cf[1].position.x = position[0] + self.initial_offset.position.x 
            each_cf[1].position.y = position[1] + self.initial_offset.position.y
            each_cf[1].position.z = position[2] + self.initial_offset.position.z

class SoloFormation(FormationType):
    def __init__(self, cf_list, offset=[0, 0, 0]):
        n_cf_supported = [1]
        super(SoloFormation, self).__init__(cf_list, n_cf_supported, offset=offset)

    def compute_initial_pose(self):
        for _, each_cf in self.cf_goals.items():
            each_cf.position.x = self.initial_offset.position.x 
            each_cf.position.y = self.initial_offset.position.y
            each_cf.position.z = self.initial_offset.position.z

def calculate_rot(start_orientation, rot):
    """Apply roatation to quaternion

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




    

    