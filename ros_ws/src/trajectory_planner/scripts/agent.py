"""
Agent Class
-----------
"""

from math import sqrt
import numpy as np
from numpy import array, dot, hstack, vstack
from numpy.linalg import norm, inv
from scipy.special import binom

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
        self.goal_dist_thres = agent_args['goal_dist_thres']
        self.goal_speed_thres = agent_args['goal_speed_thres']

        self.at_goal = False
        self.agent_idx = 0 #: Index of agent in positions
        self.n_steps = 0 #: int: Number of steps in horizon

        # For testing
        self.acc_cst = array([[0.5, 0.5, 0]]).T

        #: np.array of (6*k)x(Kmax): Position and speed trajectory at each time step.
        #  Columns: predicted [p, v], Rows: Each k
        self.states = None

        self.final_traj = None #: Agent final trajectory

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
        """Check if agent reached it's goal.

        Goal is considered reach when the agent is in a radius smaller than ``goal_dist_thres`` at
        a speed lower than ``goal_speed_thres``.


        Returns:
            bool: True if goal reached
        """
        current_position = self.states[0:3, -1]
        current_speed = self.states[3:6, -1]
        goal = self.goal.reshape(3)

        dist = norm(goal - current_position)
        speed = norm(current_speed)

        if dist < self.goal_dist_thres and speed < self.goal_speed_thres:
            self.at_goal = True

        return self.at_goal

    def check_collisions(self):
        """Check current predicted trajectory for collisions.

        1 - For all predicted trajectory, check distance of all the other agents
            2 - If distance < Rmin: In collision
                3 - If collision: Find all close agents

        Returns:
            (int): Step of collision (-1 if no collision)
            (dict of float): Close agents and their distance at collision step
        """
        collision_detected = False
        n_agents = self.all_agents_traj.shape[1]

        close_agents = {}
        collision_step = -1

        agents_dist = {}

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


                    # Faster than norm
                    scaled = dot(self.scaling_matrix_inv, predicted_pos - other_agent_pos)
                    dist = sqrt(scaled[0]**2 + scaled[1]**2 + scaled[2]**2)
                    # dist = norm(scaled)

                    agents_dist[j] = dist

                    if dist < self.r_min and not collision_detected:
                        # For step 0, check a smaller radius
                        if each_step == 0 and dist > self.r_min - 0.05:
                            break

                        collision_step = each_step
                        collision_detected = True
                        # break

            if collision_detected:
                break

        # Find all close agents at collision
        if collision_detected:
            # At collision, check distance of other agents
            for j in range(n_agents):
                if j != self.agent_idx:
                    dist = agents_dist[j]

                    if dist < self.collision_check_radius:
                        close_agents[j] = dist  # Set agent distance at collision

        return collision_step, close_agents

    def interpolate_traj(self, time_step_initial, time_step_interp):
        """Interpolate agent's trajectory using a Bezier curbe.

        Args:
            time_step_initial (float): Period between samples
            time_step_interp (float): Period between interpolation samples
        """
        # 1 - Trajectory parameters
        n_sample = self.states.shape[1] - 1
        n_sample_interp = int(n_sample*time_step_initial/time_step_interp)
        end_time = n_sample*time_step_initial
        traj_times = np.linspace(0, end_time, n_sample_interp, endpoint=False)

        # 2 - Build bezier curve
        if n_sample != 0:
            self.final_traj = np.zeros((3, n_sample_interp))

            for i in range(n_sample + 1):
                point = self.states[0:3, i].reshape(3, 1)

                self.final_traj +=\
                    binom(n_sample, i) * (1 - (traj_times/end_time))**(n_sample - i) *\
                        (traj_times/end_time)**i * point

        else:
            self.final_traj = self.states[0:3, 0]
            self.final_traj = self.final_traj.reshape(3, 1)
