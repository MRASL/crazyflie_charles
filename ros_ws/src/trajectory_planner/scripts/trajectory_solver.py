#!/usr/bin/env python

"""
Package used to find a collision less trajectory between a number of agents.

Algorithm
----------
The algorithm used is the one presented in this paper: [CIT1]_

Exemple
-------
::

    A1 = Agent([0.0, 2.0, 0.0])
    A1.set_goal([4.0, 2.0, 0.0])

    A2 = Agent([4.0, 1.9999, 0.0], goal=[0.0, 1.9999, 0.0])

    solver = TrajectorySolver([A1, A2])
    solver.set_obstacle(obstacles)

    solver.set_wait_for_input(False)
    solver.set_slow_rate(1.0)

    solver.solve_trajectories()

Run Demo Scripts
----------------
It's possible to test the trajectory algorithm by running demo scripts
(``.../trajectory_planner/tests``.)

To execute  a demo, simply uncomment the desired demonstration in ``demos.py`` -> ``demo`` and run
the script (``demo.py``).

It's possible to add new demonstrations by adding them in either: ``demos_formation.py``,
``demos_positions.py`` or ``demos_random.py``.

Benchmark Algo Performances
---------------------------
The algorithm benchmarks can be calculated using ``.../trajectory_planner/tests/performance.py``.
The script will output:

* The success rate (%)
* Total compute time (sec)
* Total travel time (sec)

Those statistics are always calculated by running the same 9 demos.

.. todo:: Script to compare and analyse results of different configurations (issue #86). With a
          graphic interface?

Solver Class
------------
"""

import numpy as np
from numpy import array, dot, hstack, vstack
from numpy.linalg import norm, inv, matrix_power
from qpsolvers import solve_qp

from trajectory_plotting import TrajPlot

IN_DEBUG = False

# Global attributes

GOAL_THRES = 0.01 # 5 cm
R_MIN = 0.45
COLL_RADIUS = 2*R_MIN

class TrajectorySolver(object):
    """To solve trajectories of all agents

    Args:
        agent_list (list of Agents): Compute trajectory of all agents in the list
        verbose (:obj:`bool`): If True, prints information

    Attributes:
        horizon_time(:obj:`float`): Horizon to predict trajectory [sec]

        step_interval(:obj:`float`): Time steps interval (h) [sec]
        steps_in_horizon(:obj:`int`): Number of time steps in horizon (k)

        interp_time_step(:obj:`float`): Interpolation time step [sec]

        k_t(:obj:`int`): Current time step
        k_max(:obj:`int`): Max time step

        at_goal(:obj:`bool`): True when all agents reached their goal
        in_collision(:obj:`bool`): True when a collision is detected between two agents

        agents(:obj:`list` of :py:class:`Agent`): List of all agents
        n_agents(:obj:`int`): Number of agents
        all_agents_traj(3k x n_agents :obj:`array`): Latest predicted trajectory of each agent over
            horizon
        agents_distances(:obj:`list` of :obj:`float`)

        kapa(:obj:`int`): For more aggressive trajectories
        error_weight(:obj:`float`): Error weigth
        effort_weight(:obj:`float`): Effort weigth
        input_weight(:obj:`float`): Input variation weigth
        relaxation_weight_p(:obj:`float`): Quadratic relaxation weigth
        relaxation_weight_q(:obj:`float`): Linear relaxation weigth
        relaxation_max_bound(:obj:`float`): Maximum relaxation, 0
        relaxation_min_bound(:obj:`float`): Minimum relaxation

        r_min(:obj:`float`): Mimimum distance between agents [m]
        a_max(:obj:`float`): Maximum acceleration of an agent [m/s^2]
        a_min(:obj:`float`): Minimum acceleration of an agent [m/s^2]
        has_fix_obstacle(:obj:`bool`): True if there are fix obstacles to avoid
        obstacle_positions(:obj:`list` of :obj:`tuple`): Coordinates of obstacles to avoid (x, y, z)

    """
    def __init__(self, agent_list=None, solver_args=None, verbose=True):
        """Init solver

        Args:
            agent_list (list of Agents): Compute trajectory of all agents in the list
            verbose (bool): If True, prints information
        """
        self.verbose = verbose

        if self.verbose:
            print "Initializing solver..."

        # Time related attributes
        self.horizon_time = solver_args['horizon_time']

        self.step_interval = solver_args['step_interval']
        self.steps_in_horizon = int(self.horizon_time/self.step_interval)

        self.interp_time_step = solver_args['interp_step']

        self.k_t = 0
        self.k_max = int(solver_args['max_time']/self.step_interval)

        self.at_goal = False
        self.in_collision = False

        # Initialize agents
        self.agents = agent_list
        self.n_agents = 0
        self.all_agents_traj = None
        self.agents_distances = []

        if self.agents is not None:
            self.initialize_agents()

        # Error weights
        self.kapa = solver_args['goal_agg']
        self.error_weight = solver_args['error_weight']
        self.effort_weight = solver_args['effort_weight']
        self.input_weight = solver_args['input_weight']

        self.relaxation_max_bound = solver_args['relax_max']
        self.relaxation_min_bound = solver_args['relax_min']
        self.relaxation_inc = solver_args['relax_inc']
        self.relaxation_weight_p = solver_args['relax_weight_sq']
        self.relaxation_weight_q = solver_args['relax_weight_lin']

        # Constraints
        self.r_min = solver_args['r_min']
        self.a_max = solver_args['max_acc']
        self.a_min = solver_args['min_acc']
        p_min = solver_args['min_pos']
        p_max = solver_args['max_pos']
        self.p_min = [p_min, p_min, 0.0]
        self.p_max = [p_max, p_max, p_max]

        self.obstacle_positions = []
        self.has_fix_obstacle = False

        self.a_min_mat = array([[self.a_min, self.a_min, self.a_min]]).T
        self.a_max_mat = array([[self.a_max, self.a_max, self.a_max]]).T
        self.p_min_mat = array([self.p_min]).T
        self.p_max_mat = array([self.p_max]).T

        # State matrix
        self.a_mat = array([[]])
        self.b_mat = array([[]])
        self.a0_mat = array([[]])
        self.lambda_mat = array([[]])
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

        self.initialize_matrices()

        # Graph parameters
        self.trajectory_plotter = TrajPlot(self.agents, self.step_interval)

        if self.verbose:
            print "Solver ready"

    # Setters methods
    def wait_for_input(self, to_wait):
        """Wait for input before next frame

        Args:
            to_wait (:obj:`bool`): If True, when for input in terminal before next frame
        """
        self.trajectory_plotter.set_wait_for_input(to_wait)

    def set_slow_rate(self, slow_rate):
        """To change rate of animation

        Args:
            slow_rate (:obj:`float`): Change rate of animation
        """
        self.trajectory_plotter.set_slow_rate(slow_rate)

    def set_arena_max(self, max_val):
        """Set x and y axes limits

        Args:
            max_val(:obj:`int`): Maximum arena value, same for x and y
        """
        self.trajectory_plotter.set_axes_limits((-0.2, max_val), (-0.2, max_val))

    def set_obstacles(self, obstacle_positions):
        """Add obstacle has an agent with a cst position

        Args:
            obstacle_positions (:obj:`list` of :obj:`tuple`): Position of each obstacle (x, y, z)
        """
        if obstacle_positions: # If not empty
            self.has_fix_obstacle = True
            self.obstacle_positions = obstacle_positions

            for each_obstacle in self.obstacle_positions:

                # Each point of the wall is considered as an agent /w cst position over horizon
                obstacle_trajectories = None
                for each_obstacle_pos in each_obstacle:
                    each_coord_array = array([each_obstacle_pos]).reshape(3, 1)
                    coord = each_coord_array
                    for _ in range(1, self.steps_in_horizon):
                        coord = vstack((coord, each_coord_array))

                    if obstacle_trajectories is None:
                        obstacle_trajectories = coord
                    else:
                        obstacle_trajectories = hstack((obstacle_trajectories, coord))

                self.all_agents_traj = hstack((self.all_agents_traj, obstacle_trajectories))

        # Update all agents informations
        for agent in self.agents:
            agent.set_all_traj(self.all_agents_traj)

    # Initialization methods
    def initialize_agents(self):
        """Initialize positions and starting trajectory of all agents
        """
        self.n_agents = len(self.agents)

        # Set idx of all agents
        for i in range(self.n_agents):
            self.agents[i].agent_idx = i

        #: 3k x n_agents array: Latest predicted trajectory of each agent over horizon
        self.all_agents_traj = np.zeros((3*self.steps_in_horizon, self.n_agents))

        for each_agent in self.agents:
            each_agent.initialize_position(self.steps_in_horizon, self.all_agents_traj)

            # Set initial trajectory
            slc = slice(0, 3)
            traj = each_agent.states[:, -1]

            p_traj = traj[slc].reshape(3, 1)
            for i in range(1, self. steps_in_horizon):
                slc = slice(i*6, i*6+3)
                p_k = traj[slc].reshape(3, 1)
                p_traj = vstack((p_traj, p_k))

            self.all_agents_traj[:, each_agent.agent_idx] = p_traj[:, 0]

    def initialize_matrices(self):
        r"""Compute matrices used to determine new states

        .. math::
            :nowrap:

            \begin{align*}
                A &= \begin{bmatrix}
                        I_3 & hI_3\\
                        0_3 & I_3
                    \end{bmatrix}\\
                \\
                B &= \begin{bmatrix}
                        \frac{h^2}{2}I_3\\
                        hI_3
                    \end{bmatrix}\\
                \\
                \Lambda &= \begin{bmatrix}
                                B           & 0_3       & ...   & 0_3\\
                                AB          & B         & ...   & 0_3\\
                                ...         & ...       & ...   & ...\\
                                A^{k-1}B    & A^{k-2}B  & ...   & B
                        \end{bmatrix}\\
                \\
                A_0 &= \begin{bmatrix}
                            A^T & (A^2)^T & ... & (A^k)^T
                    \end{bmatrix}^T\\
            \end{align*}
        """
        # Build prediction matrix
        self._build_prediction_matrices()

        # Objective functions
        self._build_objective_fct_matrices()

        # Build constraints matrix
        self._build_constraint_matrices()

    def _build_prediction_matrices(self):
        """Build all matrix used to predict trajectories
        """
        # A, 6x6
        matrix_a1 = hstack((np.eye(3), np.eye(3)*self.step_interval))
        matrix_a2 = hstack((np.zeros((3, 3)), np.eye(3)))
        self.a_mat = vstack((matrix_a1, matrix_a2)) # 6x6

        # B, 6x3
        n_dim = 6
        m_dim = 3
        self.b_mat = vstack(((self.step_interval**2/2)*np.eye(3),
                             self.step_interval*np.eye(3))) # 6x3

        # Lambda, 6k x 3k
        self.lambda_mat = np.zeros((n_dim*self.steps_in_horizon, m_dim*self.steps_in_horizon))
        rsl = slice(0, n_dim)
        self.lambda_mat[rsl, :m_dim] = self.b_mat
        for i in range(1, self.steps_in_horizon):
            rsl_p, rsl = rsl, slice(i * n_dim, (i + 1) * n_dim)
            self.lambda_mat[rsl, :m_dim] = dot(self.a_mat, self.lambda_mat[rsl_p, :m_dim])
            self.lambda_mat[rsl, m_dim : (i + 1) * m_dim] = self.lambda_mat[rsl_p, : i * m_dim]

        # A0, 6k x 6
        n_dim = m_dim = 6
        self.a0_mat = np.zeros((6, 6*self.steps_in_horizon))
        rsl = slice(0, m_dim)
        self.a0_mat[:, rsl] = self.a_mat.T
        for i in range(1, self.steps_in_horizon):
            rsl_p, rsl = rsl, slice(i * m_dim, (i + 1) * m_dim)
            self.a0_mat[:, rsl] = dot(self.a_mat, self.a0_mat[:, rsl_p].T).T
        self.a0_mat = self.a0_mat.T

        # Lambda accel, 3k x 3k
        rsl = slice(3)
        self.lambda_accel = self.lambda_mat[rsl]

        for i in range(1, self.steps_in_horizon):
            rsl = slice(i * 6, 6*i + 3) # Select top three rows of block
            rows = self.lambda_mat[rsl]
            self.lambda_accel = vstack((self.lambda_accel, rows))

        # A0 accel, 3k x 6, select first three cols of each block (of the transpose)
        a0_trans = self.a0_mat.T
        rsl = slice(3)
        self.a0_accel = a0_trans[:, rsl]
        for i in range(1, self.steps_in_horizon):
            rsl = slice(i * 6, 6*i + 3) # Select every other 3 cols
            cols = a0_trans[:, rsl]
            self.a0_accel = hstack((self.a0_accel, cols))

        self.a0_accel = self.a0_accel.T

    def _build_objective_fct_matrices(self):
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

    def _build_constraint_matrices(self):
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
        self.p_min_mat = array([self.p_min]).T
        self.p_max_mat = array([self.p_max]).T
        p_min = self.p_min_mat
        p_max = self.p_max_mat

        for _ in range(1, self.steps_in_horizon):
            self.p_min_mat = vstack((self.p_min_mat, p_min))
            self.p_max_mat = vstack((self.p_max_mat, p_max))

    def update_agents_info(self):
        """To update agents information

        Has to be called when starting position or goal of an agent changed
        """
        self.initialize_agents()
        self.initialize_matrices()

        self.k_t = 0
        self.at_goal = False
        self.in_collision = False

    # Trajectory solvers
    def solve_trajectories(self):
        """Compute trajectories and acceleration of each agent for the current time step

        Core of the algorithm

        Returns:
            :obj:`bool`: If the algorithm was succesfull
            :obj:`float`: Total trajectory time
        """
        if self.verbose:
            print "Solving trajectories..."

        # For each time step
        while not self.at_goal and self.k_t < self.k_max and not self.in_collision:
            # New trajectories
            new_trajectories = np.copy(self.all_agents_traj)

            # For each agent
            for agent in self.agents:
                # Determine acceleration input
                current_state = agent.states[0:6, -1].reshape(6, 1)
                accel_input = self.solve_accel(agent, current_state)

                if self.in_collision:
                    break # If there is a collision between two agents, break

                # If new acceleration feasible
                x_pred = self.predict_trajectory(current_state, accel_input) # Find new state
                agent.prev_input = accel_input[0:3, 0]

                # Extract predicted positions
                slc = slice(0, 3)
                p_pred = x_pred[slc, 0].reshape(3, 1)
                for n_dim in range(1, self. steps_in_horizon):
                    slc = slice(n_dim*6, n_dim*6+3)
                    x_k = x_pred[slc, 0].reshape(3, 1)
                    p_pred = vstack((p_pred, x_k))

                # Update trajectory of current agent
                agent_idx = self.agents.index(agent)
                new_trajectories[:, agent_idx] = p_pred.reshape(3*self.steps_in_horizon)

                agent.add_state(x_pred)

            self.check_goals()
            self.all_agents_traj[:, :] = new_trajectories[:, :] # Update all agents trajectories

            self.compute_agents_dist()
            self.k_t += 1

        self.print_final_positions()

        if self.at_goal:
            for each_agent in self.agents:
                each_agent.interpolate_traj(self.step_interval, self.interp_time_step)

        # TODO REMOVE
        # if not self.k_t < self.k_max:
        #     print "Max Time Reached"
        #     self.at_goal = True

        return self.at_goal, (self.k_t*self.step_interval)

    def solve_accel(self, agent, initial_state):
        """Optimize acceleration input for the horizon

        Args:
            agent (Agent)
            initial_state (array, 6x1): Initial state of the agent

        Returns:
            array 3*hor_steps x 1: Acceleration over the trajectory
        """

        # Check if there is a collision
        avoid_collision = agent.check_collisions()  #: True if trajectory in collision

        if avoid_collision and IN_DEBUG:
            print "\nTime %.2f at step %i: Agent %i" % (self.k_t*self.step_interval,\
                self.k_t, agent.agent_idx)

        # Build optimization problem: 1/2 x.T * p_mat * x + q_mat.T * x  s.t. g_mat*x <= h_mat
        try:
            p_mat, q_mat, g_mat, h_mat = self.build_optimization_matrices(agent,
                                                                          initial_state,
                                                                          avoid_collision)
        except TypeError:
            if self.verbose:
                print ''
                print "ERROR: In collision"
            return None

        # Solve optimization problem, increase max relaxation if no solution is found
        accel_input = solve_qp(p_mat, q_mat, G=g_mat, h=h_mat[:, 0], solver='quadprog')

        # cur_relaxation = self.relaxation_min_bound  # To locally increase relaxation bound
        # find_solution = True
        # while find_solution:
        #     try:
        #         accel_input = solve_qp(p_mat, q_mat, G=g_mat, h=h_mat[:, 0], solver='quadprog')
        #         find_solution = False
        #     except ValueError:
        #         if cur_relaxation > 2*self.relaxation_min_bound and avoid_collision:
        #             cur_relaxation -= self.relaxation_inc
        #             print "No solution, relaxing constraints: %.2f" % cur_relaxation
        #             n_collision = len(agent.close_agents.keys())

        #             for i in range(n_collision):
        #                 h_mat[(-1 - i*3), 0] = -cur_relaxation

        #         else:
        #             self.in_collision = True
        #             find_solution = False
        #             if self.verbose:
        #                 print ''
        #                 err_msg = ", Check max space"
        #                 if cur_relaxation < -10:
        #                     err_msg = ", Max relaxation reached"
        #                 print "ERROR: No solution in constraints %s" % err_msg
        #             return None

        if IN_DEBUG and avoid_collision:
            print "\t\t Relaxation: {}".format(accel_input[3*self.steps_in_horizon:])

        # Return acceleration
        accel_input = accel_input[0:3*self.steps_in_horizon]
        accel_input = accel_input.reshape(3*self.steps_in_horizon, 1)

        return accel_input

    def build_optimization_matrices(self, agent, initial_state, avoid_collision):
        """Build optimization matrices depending on collision state

        1/2 x.T * p_mat * x + q_mat.T * x  s.t. g_mat*x <= h_mat

        Args:
            agent (Agent)
            initial_state (array, 6x1): Initial state
            avoid_collision (bool): If trajectory is in collision

        Returns:
            list of matrix: see description
        """
        agent_goal = agent.goal
        prev_input = agent.prev_input

        if not avoid_collision:
            p_mat, q_mat, g_mat, h_mat = self.solve_accel_no_coll(initial_state,
                                                                  agent_goal,
                                                                  prev_input)

        else:
            p_mat, q_mat, g_mat, h_mat = self.solve_accel_coll(agent, initial_state)

        # If two agents are in collision
        if self.in_collision:
            return None

        return p_mat, q_mat, g_mat, h_mat

    def solve_accel_no_coll(self, initial_state, agent_goal, prev_input):
        """Compute acceleration over horizon when no collision are detected

        Args:
            initial_state (array, 6x1): Initial state
            agent_goal (array, 3x1): Agent goal
            prev_input (array, 3x1): Previous acceleration input

        Returns:
            p, q, g, h: matrices of QP problem
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

        return p_tot, q_tot, g_matrix, h_matrix

    def solve_accel_coll(self, agent, initial_state):
        """Compute acceleration over horizon when a collision is detected

        Args:
            agent (Agent)
            initial_state (array, 6x1): Initial state

        Returns:
            array, 3k x 1: Acceleration vector
        """
        # Check number of close agents
        collisions_list = agent.close_agents.keys() # list of int: Idx of all other agents colliding
        n_collisions = len(collisions_list)

        if IN_DEBUG:
            print "\t\t Close agents: {}".format(collisions_list)
            print "\t\t At step of horizon:  %i" % agent.collision_step

        # Collision at step 0 mean two agents collided
        if agent.collision_step == 0:
            if self.verbose:
                print "Agent %i in collision" % agent.agent_idx
                print "Min Distance: %.2f" % min([dist for _, dist in agent.close_agents.items()])
            self.in_collision = True
            return 0, 0, 0, 0

        # Agent start_position at collision time step
        agent_goal = np.copy(agent.goal)
        start_rows = slice(0, 3)
        agent_start_pos = self.all_agents_traj[start_rows, agent.agent_idx]
        agent_dy = agent_goal[1, 0] - agent_start_pos[1]
        agent_dx = agent_goal[0, 0] - agent_start_pos[0]
        goal_line = agent_dy/agent_dx if agent_dx != 0 else 0

        # Check if an agent is on a direct line to goal, used to avoid deadlocks
        for agent_idx in agent.close_agents:
            # Other agent position
            other_start_pos = self.all_agents_traj[start_rows, agent_idx]

            other_dx = other_start_pos[0] - agent_start_pos[0]
            other_dy = other_start_pos[1] - agent_start_pos[1]
            other_line = other_dy/other_dx if other_dx != 0 else 0

            if abs(goal_line - other_line) < 0.01:
                # If agents are not vertically aligned
                if agent_dx != 0 and other_dx != 0:
                    agent_goal[1, 0] += 0.5 if agent_dx > 0 else -0.5

                else:
                    agent_goal[0, 0] += 0.5 if agent_dy > 0 else -0.5

        # Get no collision problem
        p_no_coll, q_no_coll, g_no_coll, h_no_coll = self.solve_accel_no_coll(initial_state,
                                                                              agent_goal,
                                                                              agent.prev_input)

        # Augment matrices
        g_coll = np.c_[g_no_coll, np.zeros((g_no_coll.shape[0], n_collisions))]
        h_coll = h_no_coll
        p_aug = np.c_[p_no_coll, np.zeros((p_no_coll.shape[0], n_collisions))]
        p_aug = np.r_[p_aug, np.zeros((n_collisions, p_aug.shape[1]))]
        q_aug = np.concatenate([q_no_coll, np.zeros((n_collisions))])

        # Agent position at collision time step
        collision_rows = slice(agent.collision_step*3, (agent.collision_step+1)*3)
        agent_position_coll = self.all_agents_traj[collision_rows, agent.agent_idx]

        for agent_j_idx, dist in agent.close_agents.items():
            collision_idx = collisions_list.index(agent_j_idx)

            # Other agent position at collision time step
            other_position_coll = self.all_agents_traj[collision_rows, agent_j_idx]

            # Build matrices
            #: 3x1 array, v_ij = scaling_matrix**-2 @ (p_i - p_j)
            v_matrix = dot(matrix_power(agent.scaling_matrix, -2),
                           agent_position_coll - other_position_coll)

            #: 1x1 array, rho_ij = r_min*ksi_ik + ksi_ij**2 + v_ij' * p_i
            rho_constraint = self.r_min*dist - dist**2 + dot(v_matrix.T, agent_position_coll)

            #: 3k x 1 array
            mu_matrix = np.zeros((3*(agent.collision_step), 1))
            mu_matrix = vstack((mu_matrix, v_matrix.reshape(3, 1)))
            mu_matrix = vstack((mu_matrix,
                                np.zeros((3*(self.steps_in_horizon - agent.collision_step - 1),
                                          1))))

            # Contraintes
            accel_constraint = dot(mu_matrix.T, self.lambda_accel)[0]
            h_relaxation_const = rho_constraint - dot(mu_matrix.T,
                                                      dot(self.a0_accel, initial_state))[0, 0]

            other_agents_mat = np.zeros((n_collisions))
            other_agents_mat[collision_idx] = 1*dist

            g_relaxation_const = hstack((-1*accel_constraint, other_agents_mat))

            # Add constraints
            g_coll = vstack((g_coll, g_relaxation_const))
            h_coll = vstack((h_coll, -h_relaxation_const))

            # Add limit to e_ij
            e_bound = np.zeros((1, (3*self.steps_in_horizon) + n_collisions))
            e_bound[0, 3*self.steps_in_horizon + collision_idx] = 1

            g_coll = np.r_[g_coll, e_bound]
            g_coll = np.r_[g_coll, -1*e_bound]

            h_coll = np.r_[h_coll, array([[self.relaxation_max_bound]])]
            h_coll = np.r_[h_coll, array([[-self.relaxation_min_bound]])]

        # Add relaxation penalty
        total_size = 3*self.steps_in_horizon + n_collisions
        p_relaxation = np.zeros((total_size, total_size))
        p_relaxation[3*self.steps_in_horizon:, 3*self.steps_in_horizon:] = np.eye(n_collisions)
        p_relaxation = 2*p_relaxation*self.relaxation_weight_p

        q_relaxation = np.zeros((total_size))
        q_relaxation[3*self.steps_in_horizon:] = np.ones(n_collisions)
        q_relaxation = -1*q_relaxation * self.relaxation_weight_q

        p_coll = p_aug + p_relaxation
        q_coll = q_aug + q_relaxation

        return p_coll, q_coll, g_coll, h_coll

    def hard_constraints(self, agent, initial_state):
        """Test with hard collision constraints
        """
        # Check number of close agents
        collisions_list = agent.close_agents.keys() # list of int: Idx of all other agents colliding

        if IN_DEBUG:
            print "\t\t Collision detected with: {}".format(collisions_list)
            print "\t\t At step of horizon:  %i" % agent.collision_step

        # Collision at step 0 means two agents collided
        if agent.collision_step == 0:
            if self.verbose:
                print "Agent %i in collision" % agent.agent_idx
            self.in_collision = True
            return 0, 0, 0, 0

        # Agent start_position at collision time step
        agent_goal = np.copy(agent.goal)
        start_rows = slice(0, 3)
        agent_start_pos = self.all_agents_traj[start_rows, agent.agent_idx]
        agent_dy = agent_goal[1, 0] - agent_start_pos[1]
        agent_dx = agent_goal[0, 0] - agent_start_pos[0]
        goal_line = agent_dy/agent_dx if agent_dx != 0 else 0

        # Check if an agent is on a direct line to goal, used to avoid deadlocks
        for agent_idx in agent.close_agents:
            # Other agent position
            other_start_pos = self.all_agents_traj[start_rows, agent_idx]

            other_dx = other_start_pos[0] - agent_start_pos[0]
            other_dy = other_start_pos[1] - agent_start_pos[1]
            other_line = other_dy/other_dx if other_dx != 0 else 0

            if abs(goal_line - other_line) < 0.01:
                agent_goal[1, 0] += 0.5 if agent_dx > 0 else -0.5

        # Get no collision problem
        p_no_coll, q_no_coll, g_no_coll, h_no_coll = self.solve_accel_no_coll(initial_state,
                                                                              agent_goal,
                                                                              agent.prev_input)

        # Agent position at collision time step
        collision_rows = slice(agent.collision_step*3, (agent.collision_step+1)*3)
        agent_position_coll = self.all_agents_traj[collision_rows, agent.agent_idx]

        for agent_j_idx, dist in agent.close_agents.items():
            # Other agent position at collision time step
            other_position_coll = self.all_agents_traj[collision_rows, agent_j_idx]

            # Build matrices
            #: 3x1 array, v_ij = scaling_matrix**-2 @ (p_i - p_j)
            v_matrix = dot(matrix_power(agent.scaling_matrix, -2),
                           agent_position_coll - other_position_coll)

            #: 1x1 array
            rho_constraint = (self.r_min - dist)*dist + dot(v_matrix.T, agent_position_coll)
            # rho_constraint = (self.r_min**2 - dist**2)/2 + dot(v_matrix.T, agent_position_coll)

            #: 3k x 1 array
            mu_matrix = np.zeros((3*(agent.collision_step - 1), 1))
            mu_matrix = vstack((mu_matrix, v_matrix.reshape(3, 1)))
            mu_matrix = vstack((mu_matrix,
                                np.zeros((3*(self.steps_in_horizon - agent.collision_step),
                                          1))))

            # Contraintes
            accel_constraint = dot(mu_matrix.T, self.lambda_accel)[0]
            h_relaxation_const = rho_constraint - dot(mu_matrix.T,
                                                      dot(self.a0_accel, initial_state))[0, 0]

            # Add constraints
            g_coll = vstack((g_no_coll, -1*accel_constraint))
            h_coll = vstack((h_no_coll, -h_relaxation_const))

        return p_no_coll, q_no_coll, g_coll, h_coll

    def predict_trajectory(self, current_state, accel):
        """Predict an agent trajectory based on it's position and acceleration

        Args:
            x (np.array, 6x1): Current state
            u (np.array, 6*k x 1): Acceleration for horizon

        Returns:
            x_pred (np.array, k*6x1): Predicted states over the horizon
        """
        # Computing predicted trajectory
        x_pred = dot(self.a0_mat, current_state) + dot(self.lambda_mat, accel)

        return x_pred

    def check_goals(self):
        """Verify if all agents are in a small radius around their goal
        """
        all_goal_reached = True
        for each_agent in self.agents:
            if not each_agent.check_goal():
                all_goal_reached = False

        self.at_goal = all_goal_reached

    def compute_agents_dist(self):
        """Print distance of each agent at new position
        """

        scaling_matrix = np.diag([1, 1, 2])
        scaling_matrix_inv = inv(scaling_matrix)

        # if IN_DEBUG:
        #     print "\n\t Distances between agents"

        for i in range(self.all_agents_traj.shape[1]):
            # if IN_DEBUG:
            #     print "\t Agent %i" % i

            pos_agent = self.all_agents_traj[0:3, i]

            for j in range(self.n_agents):
                if j != i:
                    pos_col = self.all_agents_traj[0:3, j]

                    dist = norm(dot(scaling_matrix_inv, pos_agent - pos_col))
                    self.agents_distances.append(dist)

                    # if IN_DEBUG:
                    #     print "\t\t From agent %i: %.2f" % (j, dist)

    # UI and printing methods
    def print_final_positions(self):
        """Print final position of all agents
        """
        if self.verbose:
            if self.at_goal:
                print "Trajectory succesfull"
                if self.agents_distances:
                    print "Minimal distance between agents: %.2f" % min(self.agents_distances)

                for each_agent in self.agents:
                    print "Final pos, agent", self.agents.index(each_agent), ": {}".format(
                        each_agent.states[0:3, -1])

                print "Time to reach goal: %.2f" % (self.k_t*self.step_interval)

            elif self.in_collision:
                print "Trajectory failed: Collision"

            else:
                print "Trajectory failed: Max time reached"

    def plot_trajectories(self):
        """Plot all computed trajectories
        """
        if self.has_fix_obstacle:
            self.trajectory_plotter.plot_obstacle(self.obstacle_positions)

        self.trajectory_plotter.update_objects(self.agents)
        self.trajectory_plotter.run()
