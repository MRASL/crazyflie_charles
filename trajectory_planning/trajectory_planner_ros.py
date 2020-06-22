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

import ast
import rospy

from crazyflie_charles.srv import SetPositions
from trajectory_solver import TrajectorySolver, Agent

class TrajectoryPlanner(object):
    """To plan trajectories of CFs
    """
    def __init__(self, cf_list):
        #: dict of str: Agent
        self.agents = {}
        for each_cf in cf_list:
            self.agents[each_cf] = Agent()

        test_ag = Agent(start_pos=[0, 0, 0], goal=[1, 1, 1])
        self.solver = TrajectorySolver([test_ag])  # CHANGE

        # Start services
        rospy.Service('/set_planner_positions', SetPositions, self.set_positions)

    def set_positions(self, srv_req):
        """Set start position or goal of each agent

        Args:
            srv_req (SetPositions): List /w position type(start or goal) and positions of each agent
        """
        pos_type = srv_req.position_type
        cf_positions = ast.literal_eval(srv_req.positions)

        if pos_type == "start_position":
            print "Setting start positions to:"
            print cf_positions

            for cf_id, start_pos in cf_positions.items():
                self.agents[cf_id].set_starting_position(start_pos)

        elif pos_type == "goal":
            print "Setting goals to:"
            print cf_positions

            for cf_id, goal in cf_positions.items():
                self.agents[cf_id].set_goal(goal)

        else:
            rospy.logerr("Invalid position type")

        return {"success": True}

if __name__ == '__main__':
    # Launch node
    rospy.init_node('trajectory_planner', anonymous=False)

    # # Get params
    CF_LIST = rospy.get_param("~cf_list", "['cf1']")
    # TO_SIM = rospy.get_param("~to_sim", "False")

    # Initialize planner
    PLANNER = TrajectoryPlanner(CF_LIST)

    rospy.spin()
