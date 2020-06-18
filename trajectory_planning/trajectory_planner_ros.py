#!/usr/bin/env python
"""To path trajectories of CFs

Services:

Subscribed Services:

Subscribtion:

Publisher:

Etapes:
    1 - Service to set start_position and goal of all agents
        - Send msg once done
    2 - Service to start planning
        - Send msg once
    3 - Service to return all positions of time_step x
"""

import rospy

from trajectory_solver import TrajectorySolver, Agent

class TrajectoryPlanner(object):
    """To plan trajectories of CFs
    """
    def __init__(self, cf_list):
        self.cf_list = cf_list
        pass

if __name__ == '__main__':
    # Launch node
    rospy.init_node('trajectory_planner', anonymous=False)

    # # Get params
    CF_LIST = rospy.get_param("~cf_list", "['cf1']")
    # TO_SIM = rospy.get_param("~to_sim", "False")

    # Initialize planner
    PLANNER = TrajectoryPlanner(CF_LIST)

    rospy.spin()