#!/usr/bin/env python

"""To manage the formation of the swarm

In charge of computing the positions of all the crazyflies.

Notes:
    Avalaible formations (# cf):
        - Square (4, 9)
        - Circle ()
        - Triangle (3)
        - Pyramid (4)
"""

from geometry_msgs.msg import Pose
import numpy as np
import rospy

class Formation(object):
    def __init__(self, cf_list, n_cf_supported=[], offset=[0, 0, 0]):
        self.cf_list = cf_list
        self.n_cf = len(cf_list) #: (int) Number of CF in the formation
        
        self.poses = {} #: (dict of Pose) Target Pose of all the CF
        for each_cf in cf_list:
            p = Pose()
            p.orientation.w = 1
            self.poses[each_cf] = p


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

        for each_cf, position in zip(self.poses.items(), pose_list):
            each_cf[1].position.x = position[0] + self.initial_offset.position.x 
            each_cf[1].position.y = position[1] + self.initial_offset.position.y
            each_cf[1].position.z = position[2] + self.initial_offset.position.z

if __name__ == '__main__':
    # cf_list = ["cf1", "cf2", "cf3", "cf4"]
    cf_list = ["cf1", "cf2", "cf3", "cf4", "cf5", "cf6", "cf7", "cf8", "cf9"]
    square = SquareFormation(cf_list)

    square.compute_initial_pose()




    

    