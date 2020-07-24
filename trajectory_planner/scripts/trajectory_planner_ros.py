#!/usr/bin/env python
"""Package to compute collision free trajectories for each agent.

The algorithm used is based on Distributed Model Predictive Control.

.. todo:: ADD Article

Usage
-----
1. Set start position and goals of each agent to compute trajectories\
(``/set_planner_positions`` srv)

2. Start trajectory solver (``/plan_trajectories`` srv)

3. Wait for trajectory solver to be done

4. Start trajectory publishing (``/pub_trajectories`` srv)

5. Wait for trajectory publishing to be done

ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
    None

Published Topics
^^^^^^^^^^^^^^^^
/cfx/trajectory_goal(crazyflie_driver/Position)
    Position of the CF on the trajectory, at each time step

Services
^^^^^^^^
/set_planner_positions(crazyflie_charles/SetPositions)
    Set position (start or goal) of each crazyflie.
    See :py:meth:`TrajectoryPlanner.set_positions` for more information.

/plan_trajectories(`std_srvs/Empty`_)
    Start solver to find a trajectory for each crazyflie.

/pub_trajectories(`std_srvs/Empty`_)
    Call to start publishing all trajectories

Services Called
^^^^^^^^^^^^^^^
.. todo:: Add link to swarm_manager srv

/traj_found(`std_srvs/SetBool`_)
    Service called once a trajectory is found.

    `data` is True if valid trajectories are found.

    `data` is false otherwise (collision, no solution in constraints).

/traj_done(`std_srvs/Empty`_)
    Service called once the last step of the trajectory is reached.

Parameters
^^^^^^^^^^
~cf_list(str, default: ['cf1'])

TrajectoryPlanner Class
-----------------------

.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
.. _std_srvs/SetBool: http://docs.ros.org/api/std_srvs/html/srv/SetBool.html
"""


import ast
import rospy

from std_srvs.srv import Empty, SetBool
from crazyflie_driver.msg import Position
from crazyflie_charles.srv import SetPositions
from trajectory_solver import TrajectorySolver
from agent import Agent


class TrajectoryPlanner(object):
    """To plan trajectories of CFs
    """
    def __init__(self, cf_list, solver_args):
        """
        Args:
            cf_list (list of str): List of all CF in the swarm
        """
        self.agents_dict = {}
        """dict of str: dict of str: Keys are the id of the CF.
        Items are a dict containing: ``Agent``, trajectory_publisher, start_yaw"""

        agents_args = {'r_min': solver_args['r_min'],
                       'col_radius_ratio': solver_args['col_radius_ratio'],
                       'goal_thres': solver_args['goal_thres']}

        for each_cf in cf_list:
            self.agents_dict[each_cf] = {}
            self.agents_dict[each_cf]['agent'] = Agent(agents_args)
            self.agents_dict[each_cf]['trajectory_pub'] =\
                rospy.Publisher('/' + each_cf + '/trajectory_goal', Position, queue_size=1)
            self.agents_dict[each_cf]['start_yaw'] = 0

        agent_list = [agent_dict['agent'] for (_, agent_dict) in self.agents_dict.items()]
        self.solver = TrajectorySolver(agent_list, solver_args, verbose=False)

        #: bool: True if a trajectories are to be planned
        self.to_plan_trajectories = False

        #: bool: True if a trajectory has been found for current position
        self.trajectory_found = False

        #: bool: True if a trajectories are to be published
        self.to_publish_traj = False

        # Start services
        rospy.Service('/set_planner_positions', SetPositions, self.set_positions)
        rospy.Service('/plan_trajectories', Empty, self.plan_trajectories)
        rospy.Service('/pub_trajectories', Empty, self.start_publishing_srv)

        rospy.loginfo("Planner: waiting for swarm manager services")
        self.send_result = rospy.ServiceProxy("/traj_found", SetBool)
        self.traj_done = rospy.ServiceProxy("/traj_done", Empty)
        rospy.loginfo("Planner: swarm manager services found")

    # Services
    def set_positions(self, srv_req):
        """Set start position or goal of each agent

        Args:
            srv_req (SetPositions): List /w position type(start or goal) and positions of each agent
        """
        pos_type = srv_req.position_type
        cf_positions = ast.literal_eval(srv_req.positions)
        self.trajectory_found = False

        if pos_type == "start_position":
            # print "Setting start positions to:"
            # print cf_positions

            rospy.loginfo("Planner: Setting start positions")
            for cf_id, start_pos in cf_positions.items():
                start_coord = start_pos[0:3]
                start_yaw = start_pos[3]
                self.agents_dict[cf_id]['agent'].set_starting_position(start_coord)
                self.agents_dict[cf_id]['start_yaw'] = start_yaw

        elif pos_type == "goal":
            # print "Setting goals to:"
            # print cf_positions

            rospy.loginfo("Planner: Setting goals")
            for cf_id, goal in cf_positions.items():
                goal_coord = goal[0:3]
                # goal_yaw = goal[3]
                self.agents_dict[cf_id]['agent'].set_goal(goal_coord)

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

    # Methods
    def publish_trajectories(self):
        """Publish computed trajectory
        """
        # Set swarm manager to follow trajectory

        rospy.loginfo("Planner: Publishing trajectories...")
        rate = rospy.Rate(10)
        time_step = 0

        max_time_step = self.agents_dict[self.agents_dict.keys()[0]]['agent'].states.shape[1]
        while time_step < max_time_step:
            if rospy.is_shutdown():
                break

            for _, agent_dict in self.agents_dict.items():
                agent_pos = agent_dict["agent"].states[0:3, time_step]
                goal_msg = Position()

                goal_msg.x = agent_pos[0]
                goal_msg.y = agent_pos[1]
                goal_msg.z = agent_pos[2]
                goal_msg.yaw = agent_dict["start_yaw"]

                agent_dict['trajectory_pub'].publish(goal_msg)

            time_step += 1
            rate.sleep()

        rospy.loginfo("Planner: Trajectory completed")

        self.traj_done()

    def run_planner(self):
        """Execute the correct method based on booleen states
        """
        if self.to_plan_trajectories and not self.trajectory_found:
            self.to_plan_trajectories = False
            rospy.loginfo("Planner: Planning trajectories...")
            planner_successfull, _ = self.solver.solve_trajectories()

            if planner_successfull:
                self.trajectory_found = True
                rospy.loginfo("Planner: Trajectory found")
                # self.solver.plot_trajectories()
            else:
                self.trajectory_found = False
                rospy.logerr("Planner: No trajectory found")
                self.solver.plot_trajectories()

            self.send_result(self.trajectory_found)

        if self.to_publish_traj:
            self.to_publish_traj = False
            self.publish_trajectories()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('trajectory_planner', anonymous=False)

    # Get params
    while True: # Make sure cf_list has been set by `swarm_controller`
        try:
            CF_LIST = rospy.get_param("cf_list")
            break
        except KeyError:
            pass

    SOLVER_ARGS = rospy.get_param("trajectory_solver")

    # Initialize planner
    PLANNER = TrajectoryPlanner(CF_LIST, SOLVER_ARGS)

    while not rospy.is_shutdown():
        PLANNER.run_planner()
