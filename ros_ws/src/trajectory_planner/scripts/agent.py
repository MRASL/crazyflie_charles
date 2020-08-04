"""
Agent Class
-----------
"""

import numpy as np
from numpy import array, dot, hstack, vstack
from numpy.linalg import norm, inv

class Agent(object):
    """Represents a single agent
    """
    def __init__(self, agent_args, start_pos=None, goal=None):
        """Initialize agent class

        Args:
            start_pos (list of float, optional): Starting position [x, y, z]. Defaults to None.
            goal (list of float, optional): Target position [x, y, z]. Defaults to None.
        """
        # Attributes
        self.start_position = None
        self.goal = None
        self.set_starting_position(start_pos) #: 3x1 np.array: Starting position
        self.set_goal(goal) #: 3x1 np.array: Goal

        self.r_min = agent_args['r_min']
        self.collision_check_radius = self.r_min * agent_args['col_radius_ratio']
        self.goal_thres = agent_args['goal_thres']
        self.at_goal = False
        self.agent_idx = 0 #: Index of agent in positions
        self.n_steps = 0 #: int: Number of steps in horizon

        # For testing
        self.acc_cst = array([[0.5, 0.5, 0]]).T

        #: np.array of (6*k)x(Kmax): Position and speed trajectory at each time step.
        #  Columns: predicted [p, v], Rows: Each k
        self.states = None

        self.prev_input = [0, 0, 0]

        self.scaling_matrix = np.diag([1, 1, 2])
        self.scaling_matrix_inv = inv(self.scaling_matrix)

        #: (dict of int: float): Distance of each agent within a certain radius
        self.close_agents = {}
        self.collision_step = 0 #: Step of prediction where collision happens

        self.all_agents_traj = None

    # Setter
    def set_starting_position(self, position):
        """Set starting position

        Args:
            position (list of float): [x, y, z]
        """
        if position is not None:
            self.start_position = array(position).reshape(3, 1)
        else:
            self.start_position = array([0.0, 0.0, 0.0]).reshape(3, 1)

    def set_goal(self, goal):
        """Set agent goal

        Args:
            goal (list of float): [x, y, z]
        """
        if goal is not None:
            self.goal = array(goal).reshape(3, 1)

        else:
            self.goal = array([0.0, 0.0, 0.0]).reshape(3, 1)

    def set_all_traj(self, all_trajectories):
        """Set last predicted trajectories of all agents

        Args:
            all_trajectories (6*k x n_agents array)
        """
        self.all_agents_traj = all_trajectories

    def add_state(self, new_state):
        """Add new state to list of positions

        Args:
            new_state (array): Trajectory at time step
        """
        self.states = hstack((self.states, new_state))

    def get_final_traj(self):
        """Get trajectory of agent to goal

        Returns:
            3x(n time steps): Position trajectory
        """
        return self.states[0:3, :]

    # Initialization
    def initialize_position(self, n_steps, all_agents_traj):
        """Initialize position of the agent.

        Sets first horizon as a straight line to goal at a cst speed

        Args:
            n_steps (int): Number of time steps of horizon
            all_agents_traj (3k x n_agents array): Last predicted traj of each agent (ptr)
        """
        self.n_steps = n_steps
        self.all_agents_traj = all_agents_traj.view()
        self.at_goal = False

        speed = 0.1

        # Compute speeds
        dist = norm(self.goal - self.start_position)
        dist_z = norm(self.goal[2, 0] - self.start_position[2, 0])

        speed_z = dist_z * speed / dist if dist != 0 else 0
        d_speed = speed**2 - speed_z**2
        speed_xy = np.sqrt(d_speed) if d_speed > 0 else 0

        dist_x = norm(self.goal[0, 0] - self.start_position[0, 0])
        dist_y = norm(self.goal[1, 0] - self.start_position[1, 0])

        speed_x = speed_xy*dist_x/dist if dist != 0 else 0
        speed_y = speed_xy*dist_y/dist if dist != 0 else 0

        # Check signs
        if self.goal[0, 0] - self.start_position[0, 0] < 0:
            speed_x = -speed_x

        if self.goal[1, 0] - self.start_position[1, 0] < 0:
            speed_y = -speed_y

        if self.goal[2, 0] - self.start_position[2, 0] < 0:
            speed_z = -speed_z

        speed = array([[speed_x, speed_y, speed_z]]).reshape(3, 1)
        speed_position = vstack((speed, np.zeros((3, 1))))

        # Compute positions
        start_pos = vstack((self.start_position, speed))
        self.states = start_pos
        last_pos = start_pos
        for _ in range(1, self.n_steps):
            new_pos = last_pos + speed_position
            self.states = vstack((self.states, new_pos))
            last_pos = new_pos

    # Compute methods
    def check_goal(self):
        """Check if agent is in a small radius around his goal

        Returns:
            bool: True if goal reached
        """
        current_position = self.states[0:3, -1]
        goal = self.goal.reshape(3)
        dist = norm(goal - current_position)

        if dist < self.goal_thres:
            self.at_goal = True

        return self.at_goal

    def check_collisions(self):
        """Check current predicted trajectory for collisions.

        1 - For all predicted trajectory, check distance of all the other agents
            2 - If distance < Rmin: In collision
                3 - If collision: Find all close agents

        Returns:
            (bool): True if agent is in collision
        """
        collision_detected = False
        n_agents = self.all_agents_traj.shape[1]

        self.close_agents = {}

        # Find time step of collision
        for each_step in range(self.n_steps):

            # Predicted position of agent at time_step
            rows = slice(3*each_step, 3*(each_step+1))
            predicted_pos = self.all_agents_traj[rows, self.agent_idx]

            # At time step, check distance of other agents
            for j in range(n_agents):
                if j != self.agent_idx:
                    # Position of the other agent at time step
                    other_agent_pos = self.all_agents_traj[rows, j]
                    dist = norm(dot(self.scaling_matrix_inv, predicted_pos - other_agent_pos))


                    if dist < self.r_min and not collision_detected:
                        # For step 0, check a smaller radius
                        if each_step == 0 and dist > self.r_min - 0.05:
                            break

                        self.collision_step = each_step
                        collision_detected = True
                        break

            if collision_detected:
                break

        # Find all close agents at collision
        if collision_detected:
            # Predicted position of agent at time_step
            coll_rows = slice(3*self.collision_step, 3*(self.collision_step+1))
            coll_pos = self.all_agents_traj[coll_rows, self.agent_idx]

            # At collision, check distance of other agents
            for j in range(n_agents):
                if j != self.agent_idx:
                    # Position of the other agent at collision
                    other_agent_pos = self.all_agents_traj[coll_rows, j]
                    dist = norm(dot(self.scaling_matrix_inv, coll_pos - other_agent_pos))

                    if dist < self.collision_check_radius:
                        self.close_agents[j] = dist  # Set agent distance at collision

        return collision_detected

    def interpolate_traj(self, time_step_initial, time_step_interp):
        """Interpolate the trajectory for smoother paths

        Not yet implemeted.

        Args:
            time_step_initial (float): Period between samples
            time_step_interp (float): Period between interpolation samples
        """
        pass
        # n_sample = self.states.shape[1]
        # n_sample_interp = n_sample*time_step_initial/time_step_interp


        # end_time = n_sample*time_step_initial

        # traj_time = np.linspace(0, end_time, n_sample, endpoint=False)
        # traj_time_interp = np.linspace(0, end_time, n_sample_interp, endpoint=False)

        # traj_x = self.states[0, :]
        # traj_y = self.states[1, :]
        # traj_z = self.states[2, :]
