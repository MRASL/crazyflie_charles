#!/usr/bin/env python

"""
To record and analyse flight data of all CFs
"""

import os
from os.path import isfile, join
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position

class Recorder(object):
    """To record flight data of all CFs
    """
    def __init__(self, cf_list, to_save):
        self.cf_list = cf_list
        self.to_save = to_save

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
        self.crazyflies[cf_id] = {'pose': [], 'goal': []}
        rospy.Subscriber("/%s/pose" % cf_id, PoseStamped, self._cf_pose_handler, cf_id)
        rospy.Subscriber("/%s/goal" % cf_id, Position, self._cf_goal_handler, cf_id)

    def _cf_pose_handler(self, pose_stamped, cf_id):
        """Update current position of a cf

        Args:
            pose_stamped (PoseStamped): New pose of CF
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id]['pose'].append(pose_stamped)

    def _cf_goal_handler(self, goal, cf_id):
        """Update current goal of a cf

        Args:
            goal (Position): New goal of CF
            cf_id (int): Id of the CF
        """
        self.crazyflies[cf_id]['goal'].append(goal)

    def _save_data(self):
        file_path = join(self.data_path, self.data_base_name + self.data_id)
        np.save(file_path, self.crazyflies)

    def on_shutdown(self):
        """To save data upon exit
        """
        # user_cmd = raw_input("\nSave data? (y/n): ")

        # if user_cmd == 'y':
            # self._save_data()

        if self.to_save:
            self._save_data()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('flight_recorder', anonymous=False)

    while True: # Make sure cf_list has been set by `swarm_controller`
        try:
            CF_LIST = rospy.get_param("cf_list")
            break
        except KeyError:
            pass

    TO_SAVE = rospy.get_param("~to_save", "False")

    REC = Recorder(CF_LIST, TO_SAVE)
    rospy.on_shutdown(REC.on_shutdown)
    rospy.spin()
