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
from numpy.linalg import norm, inv
from qpsolvers import solve_qp
from trajectory_plotting import plot_traj
from scipy.sparse import csc_matrix

# Add a wall as an obstacle, for collision testing
wall_start = (2, 1)
wall_end = (2, 3)
wall_coords = []
wall_y = np.linspace(wall_start[1], wall_end[1], num=10)
for y in wall_y:
    wall_coords.append([wall_start[0], y, 0])

GOAL_THRES = 0.01 # 5 cm
R_MIN = 0.35
STEP_INVERVAL = 0.5
HORIZON_TIME = 1.0

# Each point of the wall is considered as an agent /w cst position over horizon
all_positions = None
for each_coord in wall_coords:
    each_coord_array = array([each_coord]).reshape(3, 1)
    coord = each_coord_array
    for each_step in range(1, int(HORIZON_TIME/STEP_INVERVAL)):
        coord = vstack((coord, each_coord_array))

    if all_positions is None:
        all_positions = coord
    else:
        all_positions = hstack((all_positions, coord))

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
        self.at_goal = False

        self.n_steps = 0 #: int: Number of steps in horizon

        # For testing
        self.acc_cst = array([[0.5, 0.5, 0]]).T

        #: np.array of (6*k)x(Kmax): Position and speed trajectorie at each time step.
        #  Columns: predicted [p, v], Rows: Each k
        self.states = None

        self.prev_input = [0, 0, 0]

        self.scaling_matrix = np.diag([1, 1, 2])
        self.scaling_matrix_inv = inv(self.scaling_matrix)

        self.collision_step = 0 #: Step of prediction where collision happens
        self.collision_idx = 0 #: int: Index of other agent 
        self.collision_dist = 0 #: float: Distance between objects at collision, defined as ksi_ij

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
        """Initialize position of the agent.

        Sets first horizon as a straight line to goal at a cst speed

        Args:
            n_steps (int): Number of time steps of horizon
        """
        self.n_steps = n_steps
        speed = 0.1

        # Compute speeds
        dist = norm(self.goal - self.start_position)
        dist_z = norm(self.goal[2, 0] - self.start_position[2,0])

        speed_z = dist_z * speed / dist
        dist_xy = np.sqrt(dist**2 - dist_z**2)
        speed_xy = np.sqrt(speed**2 - speed_z**2)

        dist_x = norm(self.goal[0, 0] - self.start_position[0,0])
        dist_y = norm(self.goal[1, 0] - self.start_position[1,0])

        speed_x = speed_xy*dist_x/dist
        speed_y = speed_xy*dist_y/dist

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

    def new_state(self, new_state):
        """Add new state to list of positions

        Args:
            new_state (array): Trajectory at time step
        """
        self.states = hstack((self.states, new_state))

    def check_goal(self):
        """Check if agent is in a small radius around his goal

        Returns:
            bool: True if goal reached
        """
        current_position = self.states[0:3, -1]
        goal = self.goal.reshape(3)
        dist = norm(goal - current_position)

        if dist < GOAL_THRES:
            self.at_goal = True

        return self.at_goal

    def check_collision(self):
        """Check current predicted trajectory for collisions.

        Returns:
            (int, int): Time step of the collision, -1 if no collision; Index of collision object
        """
        # TODO Find index of agents within a set radius

        # For each step in horizon
        for each_step in range(self.n_steps):
            predicted_pos = self.states[each_step*6: each_step*6+3, -1]
            rows = slice(3*each_step, 3*(each_step+1))
            # For each coord of the wall
            for j in range(all_positions.shape[1]): # Check all agents
                other_agent_pos = all_positions[rows, j]
                dist = norm(dot(self.scaling_matrix_inv, predicted_pos - other_agent_pos))

                if dist < R_MIN:
                    self.collision_step = each_step
                    self.collision_idx = j
                    self.collision_dist = dist
                    return True
                
        return False

class TrajectorySolver(object):
    """To solve trajectories of all agents
    """
    def __init__(self, agent_list):
        """Init solver

        Args:
            agent_list (list of Agnets): Compute trajectory of all agents in the list
        """
        self.time = 10                          # float: For testing, total time of trajectory
        self.step_interval = STEP_INVERVAL      # float: Time steps interval (h)
        self.horizon_time = HORIZON_TIME        # float: Horizon to predict trajectory
        self.k_t = 0                    # int: Current time step
        self.k_max = int(self.time/self.step_interval) # float: Maximum time
        self.at_goal = False

        #: float: Number of time steps in horizon (k)
        self.steps_in_horizon = int(self.horizon_time/self.step_interval)

        self.agents = agent_list        #: list of Agent: All agents
        self.n_agents = len(self.agents)

        # Error weights
        self.kapa = 1
        self.error_weight = 1.0
        self.effort_weight = 0.1
        self.input_weight = 0.1

        #: 3k x n_agents array: Latest predicted position of each agent over horizon
        self.all_positions = np.zeros((3*self.steps_in_horizon, self.n_agents))

        # Constraints
        self.r_min = 0.35                #: m
        self.a_max = 1.0                 #: m/s**2
        self.a_min = -1.0                #: m/s**2

        p_min = 0.0
        p_max = 10.0
        self.p_min = [p_min, p_min, p_min]
        self.p_max = [p_max, p_max, p_max]

        self.a_min_mat = array([[self.a_min, self.a_min, self.a_min]]).T
        self.a_max_mat = array([[self.a_max, self.a_max, self.a_max]]).T
        self.p_min_mat = array([self.p_min]).T
        self.p_max_mat = array([self.p_max]).T

        # State matrix
        self.A = array([[]])
        self.B = array([[]])
        self.A0 = array([[]])
        self.Lambda = array([[]])
        self.lambda_accel = array([[]])
        self.a0_accel = array([[]])

        # Objective matrix
        self.q_tilde = array([[]])
        self.r_tilde = array([[]])
        self.delta = array([[]])
        self.prev_input_mat = array([[]])
        self.s_tilde = array([[]])

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
        # Build prediction matrix
        self.build_prediction_matrix()

        # Objective functions
        self.build_objective_fct_matrix()

        # Build constraints matrix
        self.build_constraint_matrix()

    def build_prediction_matrix(self):
        """Build all matrix used to predict trajectories
        """
        # A, 6x6
        matrix_a1 = hstack((np.eye(3), np.eye(3)*self.step_interval))
        matrix_a2 = hstack((np.zeros((3, 3)), np.eye(3)))
        self.A = vstack((matrix_a1, matrix_a2)) # 6x6

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

    def build_objective_fct_matrix(self):
        """Build all matrix used in objective functions
        """
        # 1 - Trajectory error penalty
        # Q_tilde
        q_mat = self.error_weight*np.eye(3)
        self.q_tilde = np.zeros((3*self.steps_in_horizon, 3*self.steps_in_horizon))
        for i in range(self.steps_in_horizon - self.kapa, self.steps_in_horizon):
            rsl = slice(i*3, (i+1)*3)
            self.q_tilde[rsl, rsl] = q_mat

        # 2 - Control Effort penalty
        self.r_tilde = np.zeros((3*self.steps_in_horizon, 3*self.steps_in_horizon))
        r_mat = self.effort_weight*np.eye(3)
        for i in range(self.steps_in_horizon):
            rsl = slice(i*3, (i+1)*3)
            self.r_tilde[rsl, rsl] = r_mat

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

        self.prev_input_mat = np.zeros((3*self.steps_in_horizon, 1))

        self.s_tilde = np.zeros((3*self.steps_in_horizon, 3*self.steps_in_horizon))
        s_mat = self.input_weight*np.eye(3)
        for i in range(self.steps_in_horizon):
            rsl = slice(i*3, (i+1)*3)
            self.s_tilde[rsl, rsl] = s_mat

    def build_constraint_matrix(self):
        """Build all matrix used in constraints
        """
        for _ in range(1, self.steps_in_horizon):
            self.lb_constraint = vstack((self.lb_constraint, self.a_min_mat))
            self.ub_constraint = vstack((self.ub_constraint, self.a_max_mat))

        acc_min_mat = self.a_min_mat
        acc_max_mat = self.a_max_mat
        for _ in range(1, self.steps_in_horizon):
            acc_min_mat = vstack((acc_min_mat, self.a_min_mat))
            acc_max_mat = vstack((acc_max_mat, self.a_max_mat))
        
        # Add lower bound
        self.g_constraint = -np.eye(self.steps_in_horizon*3)
        self.h_constraint = -1*acc_min_mat

        # Add upper bound
        self.g_constraint = vstack((self.g_constraint, np.eye(self.steps_in_horizon*3)))
        self.h_constraint = vstack((self.h_constraint, acc_max_mat))

        # Positions constraints
        p_min = self.p_min_mat
        p_max = self.p_max_mat
        for _ in range(1, self.steps_in_horizon):
            self.p_min_mat = vstack((self.p_min_mat, p_min))
            self.p_max_mat = vstack((self.p_max_mat, p_max))

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
                accel_input = self.solve_accel(agent, current_state)

                if self.at_goal: break # FOR TESTING

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

            # self.check_goals()

            self.k_t += 1

        self.print_final_positions()

    def solve_accel(self, agent, initial_state):
        """Optimize acceleration input for the horizon

        Args:
            agent (Agent)
            initial_state (array, 6x1): Initial state of the agent

        Returns:
            array 3*hor_steps x 1: Acceleration over the trajectory
        """
        agent_goal = agent.goal
        prev_input = agent.prev_input

        if not agent.check_collision():
            accel_input = self.solve_accel_no_coll(initial_state, agent_goal, prev_input)
        
        else:
            accel_input = self.solve_accel_coll(agent, initial_state)
            self.at_goal = True

        # accel_input = self.solve_accel_no_col(initial_state, agent_goal, prev_input)
        # print accel_input
        # print "\n"

        return accel_input

    def solve_accel_no_coll(self, initial_state, agent_goal, prev_input):
        """Compute acceleration over horizon when no collision are detected

        Args:
            initial_state (array, 6x1): Initial state
            agent_goal (array, 3x1): Agent goal
            prev_input (array, 3x1): Previous acceleration input

        Returns:
            array, 3k x 1: Acceleration vector
        """
        # 1 - Trajectory error penalty
        # Goal
        goal_matrix = agent_goal
        for _ in range(1, self.steps_in_horizon):
            goal_matrix = vstack((goal_matrix, agent_goal))

        # P_e = Lambda.T * Q_tilde * Lambda
        p_error = 2 * dot(self.lambda_accel.T, dot(self.q_tilde, self.lambda_accel))
        # p_error = csc_matrix(p_error)

        q_error = -2*(dot(goal_matrix.T, dot(self.q_tilde, self.lambda_accel))-
                      dot(dot(self.a0_accel, initial_state).T,
                          dot(self.q_tilde, self.lambda_accel)))

        # 2 - Control Effort penalty
        p_effort = self.r_tilde
        q_effort = 0

        # 3 - Input Variaton penalty
        self.prev_input_mat[0:3, 0] = prev_input

        p_input = dot(self.delta.T, dot(self.s_tilde, self.delta))
        q_input = -2*dot(self.prev_input_mat.T, dot(self.s_tilde, self.delta))

        # Position constraints
        g_matrix = self.g_constraint
        h_matrix = self.h_constraint

        lb_position = self.p_min_mat - dot(self.a0_accel, initial_state)
        g_matrix = vstack((g_matrix, -1*self.lambda_accel))
        h_matrix = vstack((h_matrix, -1*lb_position))

        ub_position = self.p_max_mat - dot(self.a0_accel, initial_state)
        g_matrix = vstack((g_matrix, self.lambda_accel))
        h_matrix = vstack((h_matrix, ub_position))

        # Solve
        p_tot = p_error + p_effort + p_input
        q_tot = (q_error + q_effort + q_input).T
        q_tot = q_tot.reshape(q_tot.shape[0])  # Reshape it to work /w library

        # accel_input = solve_qp(p_tot, q_tot, solver='quadprog')
        accel_input = solve_qp(p_tot, q_tot, G=g_matrix, h=h_matrix[:, 0], solver='quadprog')

        accel_input = accel_input.reshape(3*self.steps_in_horizon, 1)

        return accel_input

    def solve_accel_coll(self, agent, initial_state):
        """Compute acceleration over horizon when a collision is detected

        Args:
            agent (Agent)
            initial_state (array, 6x1): Initial state

        Returns:
            array, 3k x 1: Acceleration vector
        """
        collision_rows = slice(agent.collision_step*3, (agent.collision_step+1)*3)
        j_position_coll = all_positions[collision_rows, agent.collision_idx]

        collision_rows = slice(agent.collision_step*6, (agent.collision_step*6)+3)
        agent_position_coll = agent.states[collision_rows, -1]

        #: 3x1 array, v_ij = scaling_matrix**-2 @ (p_i - p_j)
        v_matrix = dot(inv(dot(agent.scaling_matrix, agent.scaling_matrix)), 
                       agent_position_coll - j_position_coll)
        
        #: 3x1 array, rho_ij = r_min*ksi_ik + ksi_ij**2 + v_ij' * p_i
        rho_matrix = R_MIN*agent.collision_dist + agent.collision_dist**2 + v_matrix.T*agent_position_coll

        mu_matrix = np.zeros((3*(agent.collision_step - 1), 1))
        mu_matrix = vstack((mu_matrix, v_matrix.reshape(3, 1)))
        mu_matrix = vstack((mu_matrix, np.zeros((3*(self.steps_in_horizon - agent.collision_step), 1))))


        # TODO: Build constraint Matrix

        
        print "Collision step: %i" % agent.collision_step
        print agent_position_coll
        print j_position_coll

        print v_matrix
        print rho_matrix
        print mu_matrix

        accel_input = None


        return accel_input

    def print_final_positions(self):
        """Print final position of all agents
        """
        for each_agent in self.agents:
            print "Final pos, agent", self.agents.index(each_agent), ": {}".format(
                each_agent.states[0:2, -1])

        print "Time to reach goal: %.2f" % (self.k_t*self.step_interval)

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

    def check_goals(self):
        """Verify if all agents are in a small radius around their goal
        """
        all_goal_reached = True
        for each_agent in self.agents:
            if not each_agent.check_goal():
                all_goal_reached = False

        self.at_goal = all_goal_reached

    def plot_trajectories(self):
        """Plot all computed trajectories
        """
        plot_traj(self.agents, self.step_interval)

if __name__ == '__main__':
    START_TIME = time.time()

    A1 = Agent([0.0, 2.0, 0.0])
    A1.set_goal([4.0, 2.0, 0.0])

    A2 = Agent([1.0, 4.0, 0.0])
    A2.set_goal([1.0, 1.0, 0.0])

    A3 = Agent([3.0, 0.5, 0.0])
    A3.set_goal([2.0, 2.0, 0.0])

    A4 = Agent([4.0, 4.0, 0.0])
    A4.set_goal([0.0, 4.0, 0.0])


    SOLVER = TrajectorySolver([A1])
    # SOLVER = TrajectorySolver([A1, A2, A3, A4])

    SOLVER.solve_trajectories()
    print "Compute time:", (time.time() - START_TIME)*1000, "ms"

    SOLVER.plot_trajectories()
