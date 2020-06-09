#!/usr/bin/env python

"""Script to generate trajectories of multiple agents

Etapes:
    - Avec acceleration constantes
    1 - [x] Trajectoire pour un agent, horizon 1
    2 - [x] Plot de la trajectoire
    3 - [x] Trajectoire pour un agent, horizon > 1
    4 - [x] Plot de l'horizon
    5 - [x] Trajectoire pour plus d'un agent

    6 - [ ] Add acceleration computing, no collision, one agent
    7 - [ ] Path planning, fix object collision, one agent
    8 - [ ] Path planning, fix object collision, N agents
    9 - [ ] Path planning, collision between agents
"""

import time
import numpy as np
from numpy import array, dot, hstack, vstack
from qpsolvers import solve_qp
from trajectory_plotting import plot_traj
from scipy.sparse import csc_matrix

# Add a wall as an obstacle, for collision testing
wall_start = (2, 0)
wall_end = (2, 4)
wall_coords = []
wall_y = np.linspace(wall_start[1], wall_end[1], num=10)
for y in wall_y:
    wall_coords.append([wall_start[0], y, 0])

class Agent(object):
    """Represents a single agent
    """
    def __init__(self, start_pos=None, goal=None):
        """Initialize agent class

        Args:
            start_pos (list of float, optional): Starting position [x, y, z]. Defaults to None.
            goal (list of float, optional): Target position [x, y, z]. Defaults to None.
        """
        # Attributes
        self.start_position = array(start_pos).reshape(3, 1) #: 3x1 np.array: Starting position
        self.goal = goal #: 3x1 np.array: Goal

        # For testing
        self.acc_cst = array([[0.5, 0.5, 0]]).T

        #: np.array of (6*k)x(Kmax): Position and speed trajectorie at each time step.
        #  Columns: predicted [p, v], Rows: Each k
        self.states = None

        self.prev_input = [0, 0, 0]

    def set_starting_position(self, position):
        """Set starting position

        Args:
            position (list of float): [x, y, z]
        """
        self.start_position = array(position).reshape(3, 1)

    def set_goal(self, goal):
        """Set agent goal

        Args:
            goal (list of float): [x, y, z]
        """
        self.goal = array(goal).reshape(3, 1)

    def set_accel(self, new_accel):
        """Set constant acceleration, Only for testing purpose

        Args:
            new_accel (list of float): [ax, ay, az]
        """
        self.acc_cst = array([new_accel]).T

    def initialize_position(self, n_steps):
        """Initialize position of the agent

        Args:
            n_steps (int): Number of time steps of horizon
        """
        x_in = vstack((self.start_position, np.zeros((3, 1))))
        self.states = x_in
        for _ in range(1, n_steps):
            self.states = vstack((self.states, x_in))

    def new_state(self, new_state):
        """Add new state to list of positions

        Args:
            new_state (array): Trajectory at time step
        """
        self.states = hstack((self.states, new_state))

class TrajectorySolver(object):
    """To solve trajectories of all agents
    """
    def __init__(self, agent_list):
        """Init solver

        Args:
            agent_list (list of Agnets): Compute trajectory of all agents in the list
        """
        self.time = 5                   # float: For testing, total time of trajectory
        self.step_interval = 0.2       # float: Time steps interval (h)
        self.horizon_time = 1.0         # float: Horizon to predict trajectory
        self.k_t = 0                    # int: Current time step
        self.k_max = int(self.time/self.step_interval) # float: Maximum time
        self.at_goal = False

        #: float: Number of time steps in horizon (k)
        self.steps_in_horizon = int(self.horizon_time/self.step_interval)

        self.agents = agent_list        #: list of Agent: All agents
        self.n_agents = len(self.agents)

        #: 3k x n_agents array: Latest predicted position of each agent over horizon
        self.all_positions = np.zeros((3*self.steps_in_horizon, self.n_agents))

        # Constraints
        self.r_min = 0.35             #: m
        self.a_max = 1                #: m/s**2
        self.a_min = -1*self.a_max    #: m/s**2

        self.a_max_mat = array([[self.a_max, self.a_max, self.a_max]]).T
        self.a_min_mat = array([[self.a_min, self.a_min, self.a_min]]).T

        # State matrix
        self.A = array([[]])
        self.B = array([[]])
        self.A0 = array([[]])
        self.Lambda = array([[]])

        # Accel matrix
        self.lambda_accel = array([[]])
        self.a0_accel = array([[]])
        self.q_tilde = array([[]])
        self.r_tilde = array([[]])
        self.delta = array([[]])

        # Constraint matrix
        self.h_constraint = array([[]])
        self.g_constraint = array([[]])
        self.lb_constraint = self.a_min_mat
        self.ub_constraint = self.a_max_mat

        self.initialize_matrix()

    def initialize_matrix(self):
        """Compute matrix used to determine new states

        Notes:
            A = | I3    h*I3 |
                | 03    I3   |

            B = | (h**2/2)I3 |
                |    h*I3    |

                     |     B           03       ..  03 |
            Lambda = |     AB           B       ..  03 |
                     | ..                              |
                     | A**(k-1)B    A**(k-2)B   ..  B  |

            A0 = | A.T  A**2.T  ... (A**k).T |.T
        """

        # A, 6x6
        matrix_A1 = hstack((np.eye(3), np.eye(3)*self.step_interval))
        matrix_A2 = hstack((np.zeros((3, 3)), np.eye(3)))
        self.A = vstack((matrix_A1, matrix_A2)) # 6x6

        # B, 6x3
        N = 6
        M = 3
        self.B = vstack(((self.step_interval**2/2)*np.eye(3),
                         self.step_interval*np.eye(3))) # 6x3

        # Lambda, 6k x 3k
        self.Lambda = np.zeros((N*self.steps_in_horizon, M*self.steps_in_horizon))
        rsl = slice(0, N)
        self.Lambda[rsl, :M] = self.B
        for i in range(1, self.steps_in_horizon):
            rsl_p, rsl = rsl, slice(i * N, (i + 1) * N)
            self.Lambda[rsl, :M] = dot(self.A, self.Lambda[rsl_p, :M])
            self.Lambda[rsl, M : (i + 1) * M] = self.Lambda[rsl_p, : i * M]

        # A0, 6k x 6
        N = M = 6
        self.A0 = np.zeros((6, 6*self.steps_in_horizon))
        rsl = slice(0, M)
        self.A0[:, rsl] = self.A.T
        for i in range(1, self.steps_in_horizon):
            rsl_p, rsl = rsl, slice(i * M, (i + 1) * M)
            self.A0[:, rsl] = dot(self.A, self.A0[:, rsl_p].T).T
        self.A0 = self.A0.T

        # Build constraints matrix
        
        for _ in range(1, self.steps_in_horizon):
            self.lb_constraint = vstack((self.lb_constraint, self.a_min_mat))
            self.ub_constraint = vstack((self.ub_constraint, self.a_max_mat))

        # acc_constraint = vstack((self.a_max_mat, -1*self.a_min_mat))

        # self.h_constraint = acc_constraint
        # for _ in range(1, self.steps_in_horizon):
        #     self.h_constraint = vstack((self.h_constraint, acc_constraint))
        # # self.h_constraint = csc_matrix(self.h_constraint)

        # self.g_constraint = np.zeros((6*self.steps_in_horizon, 3*self.steps_in_horizon))
        # g_mat = vstack((np.eye(3), 1*np.eye(3)))
        # for i in range(self.steps_in_horizon):
        #     rows = slice(6*i, (i+1)*6)
        #     cols = slice(3*i, (i+1)*3)
        #     self.g_constraint[rows, cols] = g_mat

        # self.g_constraint = csc_matrix(self.g_constraint)

    def initialize(self):
        """Initialize positions and starting trajectory of all agents
        """
        for each_agent in self.agents:
            each_agent.initialize_position(self.steps_in_horizon)

        # TODO: Init all_positions as a straight line?

    def solve_trajectories(self):
        """Compute trajectories and acceleration of each agent for the current time step

        Core of the algorithm
        """
        # Initialisation
        self.initialize()

        # For each time step
        while not self.at_goal and self.k_t < self.k_max:
            
            # For each agent
            for agent in self.agents:
                # Determine acceleration input
                current_state = agent.states[0:6, -1].reshape(6, 1)
                accel_input = self.solve_accel(current_state, agent.goal, agent.prev_input)

                # If new acceleration feasible
                x_pred = self.predict_trajectory(current_state, accel_input) # Find new state
                agent.prev_input = accel_input[0:3, 0]

                # Extract predicted positions
                slc = slice(0, 3)
                p_pred = x_pred[slc, 0].reshape(3, 1)
                for n in range(1, self. steps_in_horizon):
                    slc = slice(n*6, n*6+3)
                    x_k = x_pred[slc, 0].reshape(3, 1)
                    p_pred = vstack((p_pred, x_k))

                # Update all_positions
                agent_idx = self.agents.index(agent)
                self.all_positions[:, agent_idx] = p_pred.reshape(3*self.steps_in_horizon)

                agent.new_state(x_pred)

            self.k_t += 1

        self.print_final_positions()

    def solve_accel(self, initial_state, agent_goal, prev_input):
        """Optimize acceleration input for the horizon

        Args:
            initial_state (array, 6x1): Initial state of the agent
            agent_goal (array, 3x1): Goal of the agent

        Returns:
            array 3*hor_steps x 1: Acceleration over the trajectory
        """
        # 1 - Trajectory error penalty
        # Matrix: U, Lambda, Q_tilde, Q, P_d, X_0
        # Lambda accel, 3k x 3k
        rsl = slice(3)
        self.lambda_accel = self.Lambda[rsl]
   
        for i in range(1, self.steps_in_horizon):
            rsl = slice(i * 6, 6*i + 3) # Select top three rows of block
            rows = self.Lambda[rsl]
            self.lambda_accel = vstack((self.lambda_accel, rows))

        # A0 accel, 3k x 6, select first three cols of each block (of the transpose)
        a0_trans = self.A0.T
        rsl = slice(3)
        self.a0_accel = a0_trans[:, rsl]
        for i in range(1, self.steps_in_horizon):
            rsl = slice(i * 6, 6*i + 3) # Select every other 3 cols
            cols = a0_trans[:, rsl]
            self.a0_accel = hstack((self.a0_accel, cols))

        self.a0_accel = self.a0_accel.T

        # Q_tilde
        kapa = 1
        q_mat = np.eye(3) # TODO Values of Q?
        self.q_tilde = np.zeros((3*self.steps_in_horizon, 3*self.steps_in_horizon))
        for i in range(self.steps_in_horizon - kapa, self.steps_in_horizon):
            rsl = slice(i*3, (i+1)*3)
            self.q_tilde[rsl, rsl] = q_mat

        # Goal
        goal_matrix = agent_goal
        for _ in range(1, self.steps_in_horizon):
            goal_matrix = vstack((goal_matrix, agent_goal))

        # P_e = Lambda.T * Q_tilde * Lambda
        p_error = dot(dot(self.lambda_accel.T, self.q_tilde), self.lambda_accel)
        p_error = csc_matrix(p_error)

        q_error = -2*(dot(goal_matrix.T, dot(self.q_tilde, self.lambda_accel))-
                      dot(dot(self.a0_accel, initial_state).T,
                          dot(self.q_tilde, self.lambda_accel)))

        q_error = q_error.T
        q_error = q_error.reshape(q_error.shape[0])

        # 2 - Control Effort penalty
        self.r_tilde = np.zeros((3*self.steps_in_horizon, 3*self.steps_in_horizon))
        r_mat = np.eye(3) # TODO Values of R
        for i in range(self.steps_in_horizon):
            rsl = slice(i*3, (i+1)*3)
            self.r_tilde[rsl, rsl] = r_mat
        p_effort = self.r_tilde

        # 3 - Input Variaton penalty
        self.delta = np.zeros((3*self.steps_in_horizon, 3*self.steps_in_horizon))
        
        slc = slice(3)
        self.delta[slc, slc] = np.eye(3)
        for i in range(self.steps_in_horizon - 1):  # For all rows
            rows = slice(3*(i+1), (i+2)*3)
            cols_neg = slice(i*3, (i+1)*3)
            cols_pos = slice((i+1)*3, (i+2)*3)
            self.delta[rows, cols_neg] = -1 * np.eye(3)
            self.delta[rows, cols_pos] = np.eye(3)

        prev_input_mat = np.zeros((3*self.steps_in_horizon, 1))
        prev_input_mat[0:3, 0] = prev_input

        #TODO Build P and Q for input variation penalty


        # Solve
        p_tot = p_error
        q_tot = q_error

        accel_input = solve_qp(p_tot, q_tot,
                               lb=self.lb_constraint,
                               ub=self.ub_constraint,
                               solver='osqp')
        # accel_input = solve_qp(P, q, G=self.g_constraint, h=self.h_constraint, solver='osqp')

        accel_input = accel_input.reshape(3*self.steps_in_horizon, 1)
        return accel_input

    def print_final_positions(self):
        """Print final position of all agents
        """
        for each_agent in self.agents:
            print "Final pos, agent", self.agents.index(each_agent), ": {}".format(
                each_agent.states[0:2, -1])

    def predict_trajectory(self, current_state, accel):
        """Predict an agent trajectory based on it's position and acceleration

        Args:
            x (np.array, 6x1): Current state
            u (np.array, 6*k x 1): Acceleration for horizon

        Returns:
            x_pred (np.array, k*6x1): Predicted states over the horizon
        """
        # Computing predicted trajectory
        x_pred = dot(self.A0, current_state) + dot(self.Lambda, accel)

        return x_pred

    def plot_trajectories(self):
        """Plot all computed trajectories
        """
        plot_traj(self.agents, self.step_interval)

if __name__ == '__main__':
    START_TIME = time.time()

    A1 = Agent([0.0, 2.0, 0.0])
    A1.set_goal([4.0, 4.0, 0.0])

    A2 = Agent([1.0, 4.0, 0.0])
    A2.set_goal([1.0, 1.0, 0.0])

    A3 = Agent([3.0, 0.5, 0.0])
    A3.set_goal([2.0, 2.0, 0.0])

    A4 = Agent([4.0, 4.0, 0.0])
    A4.set_goal([0.0, 4.0, 0.0])


    # SOLVER = TrajectorySolver([A1])
    SOLVER = TrajectorySolver([A1, A2, A3, A4])

    SOLVER.solve_trajectories()
    print "Compute time:", (time.time() - START_TIME)*1000, "ms"

    SOLVER.plot_trajectories()