#!/usr/bin/env python

"""
To record and analyse flight data of all CFs
"""

import rospy
from geometry_msgs.msg import PoseStamped

class Recorder(object):
    """To record flight data of all CFs
    """
    def __init__(self, cf_list):
        self.cf_list = cf_list

        self.crazyflies = {} #: dict: Keys are name of the CF

        # Initialize each Crazyflie
        for each_cf in cf_list:
            self._init_cf(each_cf)

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

    def on_shutdown(self):
        """To save data upon exit
        """
        print "SHUTDOWN"
        print self.crazyflies


if __name__ == '__main__':
    # Launch node
    rospy.init_node('flight_recorder', anonymous=False)

    CF_LIST = rospy.get_param("~cf_list", "['cf1']")

    # Initialize swarm
    REC = Recorder(CF_LIST)

    rospy.on_shutdown(REC.on_shutdown)

    while not rospy.is_shutdown():
        pass
