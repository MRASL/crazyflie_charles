#!/usr/bin/env python
"""To path trajectories of CFs

Services:

Subscribed Services:

Subscribtion:

Publisher:

Etapes:
    1 - Service to set start_position and goal of all agents
    2 - Service to start planning
        - Send msg once
    3 - Service to return all positions of time_step x
"""

import ast
import rospy

from std_srvs.srv import Empty, SetBool
from crazyflie_driver.msg import Position
from crazyflie_charles.srv import SetPositions
from trajectory_solver import TrajectorySolver, Agent


class TrajectoryPlanner(object):
    """To plan trajectories of CFs
    """
    def __init__(self, cf_list):
        #: dict of str: dict of str: Agent, traj_pub
        self.agents = {}
        for each_cf in cf_list:
            self.agents[each_cf] = {}
            self.agents[each_cf]['agent'] = Agent()
            self.agents[each_cf]['trajectory_pub'] =\
                rospy.Publisher('/' + each_cf + '/trajectory_goal', Position, queue_size=1)

        agent_list = [agent_dict['agent'] for (_, agent_dict) in self.agents.items()]
        self.solver = TrajectorySolver(agent_list)

        self.to_plan_trajectories = False
        self.trajectory_found = False #: bool: If a trajectory has been found for current position

        self.to_publish_traj = False

        # Start services
        rospy.Service('/set_planner_positions', SetPositions, self.set_positions)
        rospy.Service('/plan_trajectories', Empty, self.plan_trajectories)
        rospy.Service('/pub_trajectories', Empty, self.start_publishing_srv)

        rospy.loginfo("Planner: waiting for swarm manager services")
        self.follow_traj = rospy.ServiceProxy("/follow_traj", Empty)
        self.traj_done = rospy.ServiceProxy("/in_formation", Empty)
        self.send_result = rospy.ServiceProxy("/traj_found", SetBool)
        rospy.loginfo("Planner: swarm manager services found")

    def set_positions(self, srv_req):
        """Set start position or goal of each agent

        Args:
            srv_req (SetPositions): List /w position type(start or goal) and positions of each agent
        """
        pos_type = srv_req.position_type
        cf_positions = ast.literal_eval(srv_req.positions)
        self.trajectory_found = False

        if pos_type == "start_position":
            print "Setting start positions to:"
            print cf_positions

            for cf_id, start_pos in cf_positions.items():
                self.agents[cf_id]['agent'].set_starting_position(start_pos)

        elif pos_type == "goal":
            print "Setting goals to:"
            print cf_positions

            for cf_id, goal in cf_positions.items():
                self.agents[cf_id]['agent'].set_goal(goal)

        else:
            rospy.logerr("Invalid position type")

        return {"success": True}

    def plan_trajectories(self, _):
        """Start trajectory planning of each agent
        """
        self.solver.update_agents_info()

        if not self.to_plan_trajectories and not self.trajectory_found:
            self.to_plan_trajectories = True

        elif self.to_plan_trajectories:
            rospy.logwarn("Solver already running")

        elif self.trajectory_found:
            rospy.logwarn("Trajectory already found for current positions")

        return {}

    def start_publishing_srv(self, _):
        """Start publishing CF trajectories
        """
        self.to_publish_traj = True

        return {}

    def publish_trajectories(self):
        """Publish computed trajectory
        """
        self.follow_traj()

        rate = rospy.Rate(10)
        time_step = 0

        max_time_step = self.agents[self.agents.keys()[0]]['agent'].states.shape[1]
        while time_step < max_time_step:
            if rospy.is_shutdown():
                break

            for _, agent_dict in self.agents.items():
                agent_pos = agent_dict["agent"].states[0:3, time_step]
                goal_msg = Position()

                goal_msg.x = agent_pos[0]
                goal_msg.y = agent_pos[1]
                goal_msg.z = agent_pos[2]

                agent_dict['trajectory_pub'].publish(goal_msg)

            time_step += 1
            rate.sleep()

        self.traj_done()

    def run_planner(self):
        """To plan trajectories
        """
        if self.to_plan_trajectories and not self.trajectory_found:
            self.to_plan_trajectories = False
            rospy.loginfo("Planner: Planning trajectories...")
            planner_successfull = self.solver.solve_trajectories()

            if planner_successfull:
                self.trajectory_found = True
                rospy.loginfo("Planner: Found trajectory")
            else:
                self.trajectory_found = False
                rospy.logerr("Planner: No trajectory found")

            self.send_result(self.trajectory_found)

        if self.to_publish_traj:
            self.publish_trajectories()
            self.to_publish_traj = False

if __name__ == '__main__':
    # Launch node
    rospy.init_node('trajectory_planner', anonymous=False)

    # # Get params
    CF_LIST = rospy.get_param("~cf_list", "['cf1']")

    # Initialize planner
    PLANNER = TrajectoryPlanner(CF_LIST)

    while not rospy.is_shutdown():
        PLANNER.run_planner()
