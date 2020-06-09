#!/usr/bin/env python

"""Script to generate trajectories of multiple agents

Etapes:
    - Avec acceleration constantes
    1 - [x] Trajectoire pour un agent, horizon 1
    2 - [x] Plot de la trajectoire
    3 - [x] Trajectoire pour un agent, horizon > 1
    4 - [x] Plot de l'horizon
    5 - [x] Trajectoire pour plus d'un agent

    6 - [ ] Determiner la meilleur accel, sans collision
    7 - [ ] Determiner s'il a des collisions
    8 - [ ] Optimiser l'accel avec collision

"""
import time
import numpy as np
from numpy import array, dot, hstack, vstack
from qpsolvers import solve_qp
from trajectory_plotting import plot_traj


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
        self.positions_data = None

    def set_starting_position(self, position):
        """Set starting position

        Args:
            position (list of float): [x, y, z]
        """
        self.start_position = array(position).reshape(3, 1)

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
        self.positions_data = x_in
        for _ in range(1, n_steps):
            self.positions_data = vstack((self.positions_data, x_in))

    def new_state(self, new_state):
        """Add new state to list of positions

        Args:
            new_state (array): Trajectory at time step
        """
        self.positions_data = hstack((self.positions_data, new_state))

class TrajectorySolver(object):
    """To solve trajectories of all agents
    """
    def __init__(self, agent_list):
        """Init solver

        Args:
            agent_list (list of Agnets): Compute trajectory of all agents in the list
        """
        self.time = 3                   # float: For testing, total time of trajectory
        self.step_interval = 0.1        # float: Time steps interval (h)
        self.horizon_time = 1.0         # float: Horizon to predict trajectory
        self.k_t = 0                    # int: Current time step
        self.k_max = int(self.time/self.step_interval) # float: Maximum time

        #: float: Number of time steps in horizon (k)
        self.steps_in_horizon = int(self.horizon_time/self.step_interval)

        self.agents = agent_list        #: list of Agent: All agents
        self.n_agents = len(self.agents)

        #: 3k x n_agents array: Latest predicted position of each agent over horizon
        self.all_positions = np.zeros((3*self.steps_in_horizon, self.n_agents))

        self.A = array([[]])
        self.B = array([[]])
        self.A0 = array([[]])
        self.Lambda = array([[]])
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
        A1 = hstack((np.eye(3), np.eye(3)*self.step_interval))
        A2 = hstack((np.zeros((3, 3)), np.eye(3)))
        self.A = vstack((A1, A2)) # 6x6

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

        # A0, 6x6k
        N = M = 6
        self.A0 = np.zeros((6, 6*self.steps_in_horizon))
        rsl = slice(0, M)
        self.A0[:, rsl] = self.A.T
        for i in range(1, self.steps_in_horizon):
            rsl_p, rsl = rsl, slice(i * M, (i + 1) * M)
            self.A0[:, rsl] = dot(self.A, self.A0[:, rsl_p].T).T
        self.A0 = self.A0.T

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
        for _ in range(self.k_max):

            # For each agent
            for agent in self.agents:
                # Determine acceleration
                # a_cur = a_pred[i*3: i*3+3, 0].reshape(3, 1) # Cst acceleration for testing
                a_cur = agent.acc_cst
                x_cur = agent.positions_data[0:6, -1].reshape(6, 1)

                # If new acceleration feasible
                x_pred = self.predict_trajectory(x_cur, a_cur) # Find new state

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

    def build_and_solve_qp(self):
        """Build and solve Quadratic Problem  """

        # Exemple:
        M = array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
        P = dot(M.T, M)  # quick way to build a symmetric matrix
        q = dot(array([3., 2., 3.]), M).reshape((3,))
        G = array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
        h = array([3., 2., -2.]).reshape((3,))
        A = array([1., 1., 1.])
        b = array([1.])

        x = solve_qp(P, q, G, h, A, b)
        print "QP solution: x = {}".format(x)

    def print_final_positions(self):
        """Print final position of all agents
        """
        for each_agent in self.agents:
            print "Final pos, agent", self.agents.index(each_agent), ": {}".format(
                each_agent.positions_data[0:2, -1])

    def plot_trajectories(self):
        """Plot all computed trajectories
        """
        plot_traj(self.agents, self.step_interval)

    def predict_trajectory(self, x, u):
        """Predict an agent trajectory based on it's position and acceleration

        Args:
            x (np.array, 6x1): Current state
            u (np.array, 3x1): Predicted acceleration

        Returns:
            x_pred (np.array, k*6x1): Predicted states over the horizon
        """

        # Build acc matrix, assuming accc is constant over horizon
        U = u
        for _ in range(1, self.steps_in_horizon):
            U = vstack((U, u))

        # Computing predicted trajectory
        X_pred = dot(self.A0, x) + dot(self.Lambda, U)

        return X_pred

if __name__ == '__main__':
    START_TIME = time.time()

    A1 = Agent([0.0, 0.0, 0.0])
    A1.set_accel([0.5, 0, 0])

    A2 = Agent([4.0, 4.0, 0.0])
    A2.set_accel([0, -0.5, 0])

    A3 = Agent([0.0, 4.0, 0.0])
    A3.set_accel([1, -1, 0])

    SOLVER = TrajectorySolver([A1, A2, A3])
    SOLVER.solve_trajectories()
    print "Compute time:", (time.time() - START_TIME)*1000, "ms"

    SOLVER.plot_trajectories()
