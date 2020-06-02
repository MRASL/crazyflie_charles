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

from geometry_msgs.msg import Pose, Quaternion
import numpy as np
import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply

class Formation(object):
    def __init__(self, cf_list, n_cf_supported=[], offset=[0, 0, 0]):
        self.cf_list = cf_list
        self.n_cf = len(cf_list) #: (int) Number of CF in the formation
        
        self.cf_goals = {} #: (dict of Pose) Target Pose of all the CF
        for each_cf in cf_list:
            p = Pose()
            p.orientation.w = 1
            self.cf_goals[each_cf] = p

        self.swarm_goal = Pose()
        self.swarm_goal.orientation.w = 1

        self.n_cf_supported = n_cf_supported #: (list of int) Different number of CF possible for each formation
        self.initial_offset = Pose() #: (Pose): Offset of the center of the formation from 0,0,0
        self.initial_offset.position.x = offset[0]
        self.initial_offset.position.y = offset[1]
        self.initial_offset.position.z = offset[2]

        self.check_n()

    def check_n(self):
        if self.n_cf in self.n_cf_supported and self.n_cf > 0:
            rospy.loginfo("Formation made of %i crazyflies" % self.n_cf)
        else:
            rospy.logerr("Unsuported number of CFs")

    def get_n_cf(self):
        return self.n_cf

    def compute_initial_pose(self):
        pass
    
    def compute_goal(self, goal_vel):
        self.swarm_goal.position.x += goal_vel.linear.x
        self.swarm_goal.position.y += goal_vel.linear.y
        self.swarm_goal.position.z += goal_vel.linear.z
        self.swarm_goal.orientation = calculate_rot(self.swarm_goal.orientation, goal_vel.angular)
                
        for _, pose in self.cf_goals.items(): 
            pose.position.x += goal_vel.linear.x
            pose.position.y += goal_vel.linear.y
            pose.position.z += goal_vel.linear.z

            pose.orientation = calculate_rot(pose.orientation, goal_vel.angular)

    def compute_new_pose(self):
        pass

    def set_offset(self, x , y, z):
        self.initial_offset.position.x = x
        self.initial_offset.position.y = y
        self.initial_offset.position.z = z

class SquareFormation(Formation):
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

class SingleFormation(Formation):
    def __init__(self, cf_list, offset=[0, 0, 0]):
        n_cf_supported = [1]
        super(SingleFormation, self).__init__(cf_list, n_cf_supported, offset=offset)

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


# if __name__ == '__main__':
#     # cf_list = ["cf1", "cf2", "cf3", "cf4"]
#     cf_list = ["cf1", "cf2", "cf3", "cf4", "cf5", "cf6", "cf7", "cf8", "cf9"]
#     square = SquareFormation(cf_list)

#     square.compute_initial_pose()




    

    