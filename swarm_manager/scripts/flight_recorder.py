#!/usr/bin/env python

"""
To record and analyse flight data of all CFs
"""

import os
from os.path import isfile, join
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

class Recorder(object):
    """To record flight data of all CFs
    """
    def __init__(self, cf_list):
        self.cf_list = cf_list
        self.data_id = '0'
        self.data_path = None
        self.data_base_name = 'flight_data_'

        self._find_id()

        self.crazyflies = {} #: dict: Keys are name of the CF

        # Initialize each Crazyflie
        for each_cf in cf_list:
            self._init_cf(each_cf)

    def _find_id(self):
        # List all data files
        parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.data_path = os.path.join(parentdir, 'flight_data')
        data_list = [f for f in os.listdir(self.data_path) if isfile(join(self.data_path, f))]

        # Find all unnamed data
        if data_list:
            data_list = [f for f in data_list if f.startswith(self.data_base_name)]

            if data_list:
                data_list = [f.replace(self.data_base_name, '') for f in data_list]
                data_list = [int(f.replace('.npy', '')) for f in data_list]
                self.data_id = str(np.max(data_list) + 1)

    def _init_cf(self, cf_id):
        """Initialize each CF

        Args:
            cf_id (str): Name of the CF
        """
        self.crazyflies[cf_id] = []
        rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self._cf_pose_handler, cf_id)

    def _cf_pose_handler(self, pose_stamped, cf_id):
        """Update current position of a cf

        Args:
            pose_stamped (PoseStamped): New pose of CF
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id].append(pose_stamped)

    def _save_data(self):
        file_path = join(self.data_path, self.data_base_name + self.data_id)
        np.save(file_path, self.crazyflies)

    def on_shutdown(self):
        """To save data upon exit
        """
        self._save_data()
        print "SHUTDOWN"
        # print self.crazyflies


if __name__ == '__main__':
    # Launch node
    rospy.init_node('flight_recorder', anonymous=False)

    CF_LIST = rospy.get_param("~cf_list", "['cf1']")

    REC = Recorder(CF_LIST)
    rospy.on_shutdown(REC.on_shutdown)
    rospy.spin()
