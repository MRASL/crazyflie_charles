#!/usr/bin/env python

"""Script to generate trajectories of multiple agents

Etapes:
    - Avec acceleration constantes
    1 - [x] Trajectoire pour un agent, horizon 1
    2 - [x] Plot de la trajectoire
    3 - [x] Trajectoire pour un agent, horizon > 1
    4 - [x] Plot de l'horizon 
    5 - [ ] Trajectoire pour plus d'un agent
    
    6 - [ ] Determiner la meilleur accel, sans collision
    7 - [ ] Determiner s'il a des collisions
    8 - [ ] Optimiser l'accel avec collision
    
TODO:
    *
"""
import numpy as np
from numpy import array, dot, transpose, hstack, vstack
from qpsolvers import solve_qp
from trajectory_plotting import plot_traj

# Test pour une trajectoire, sans collision
pos_initial = array([1., 1., 1.])
pose_final = array([2., 2., 2.])

# Variables globales
time = 3 # secondes, pour les tests
h = 0.1 # Seconds per time step
Kmax = int(time/h)
horizon_time = 1
k = int(horizon_time/h) # Horizon prediction

def algo():
    """Algorithme
    
    In: Initial and final positions (p0, pf)
    Out: Positon, Velocity and acceleration trajectories

    Variables:
        S: Concatenation of the lastest predicted positions for all agents
        x_i[k]: Positions and velocity of agent i at time step k
        a_i[k]: Acceleration of agent i at time step k


    Code:
    Init all predictions(p0, pf)
        Set S as a straight line to goal
        set all x[0]

    kt = 0, at_goal = False

    while not at_goal and kt < Kmax:
        for each_agent:
            a_i[k|kt]_pred = Build&SolveQP(x_i[kt], a_i[kt - 1], S)   # Check for collision

            if QP feasible:
                x_i[k+1|kt]_pred = GetStates(x_i[kt], a_i[k|kt]_pred)
                S_i = p_i[k+1|kt]_pred
                x_i[kt+1|kt], a_i[kt] = x_i[k|kt]_pred, a_i[0|kt]_pred

        checkGoal
        kt += 1

    # algo pour optimiser le goal...

    return [p, v, a]
    
    """
    n_agents = 1
    # Positions
    pose_initial = array([0.5, 0.5, 0.0]).reshape(3, 1)
    # pose_final = array([2., 2., 2.])  # Commencer par fixer l'accel

    all_positions = np.zeros((3*k, n_agents)) # Latest predicted position of each agent over horizon
    
    k_t = 0

    # Initialisation
    x_in = vstack((pose_initial, np.zeros((3, 1))))
    x = x_in
    for _ in range(1, k):
        x = vstack((x, x_in))
    """list of float: Position and speed trajectorie at each time step. structure: [x0, y0, z0, vx0, vy0, vz0; x1, y1, z1, vx1, vy1, vz1; ...].T """
    
    # a_pred = array([[0.1, 0, 0, 0.1, 0, 0, 0.1, 0, 0, 0.1, 0, 0, 0.1, 0, 0, 0.1, 0, 0]]).T
    a_cst = array([[0.5, 0.5, 0]]).T

    for i in range(Kmax):
        # Determine acceleration
        # a_cur = a_pred[i*3: i*3+3, 0].reshape(3, 1) # Cst acceleration for testing
        a_cur = a_cst
        x_cur = x[0:6, -1].reshape(6, 1)
    
        # If new acceleration feasible
        x_pred = get_states(x_cur, a_cur) # Find new state
        # print x_pred

        slc = slice(0, 3)
        p_pred = x_pred[slc, 0].reshape(3, 1)
        for n in range(1, k):
            slc = slice(n*6, n*6+3)
            x_k = x_pred[slc, 0].reshape(3, 1)
            p_pred = vstack((p_pred, x_k))

        all_positions[:, 0] = p_pred.reshape(3*k)
        x = hstack((x, x_pred))

        k_t += 1    

    return x

def get_states(x, u):
    """Calculate the new state based on accel input

    Args:
        x (np.array, 6x1): Current state
        u (np.array, 3x1): Predicted acceleration

    Returns:
        x_pred (np.array, k*6x1): Predicted states over the horizon 
    """
    N = 6
    M = 3

    A1 = hstack((np.eye(3), np.eye(3)*h))
    A2 = hstack((np.zeros((3, 3)), np.eye(3)))
    A = vstack((A1, A2)) # 6x6

    B = vstack( ( (h**2/2)*np.eye(3), h*np.eye(3) ) ) # 6x3

    # Build Lambda matrix
    Lambda = np.zeros((N*k, M*k)) # 6k x 3k
    rsl = slice(0, N)
    Lambda[rsl, :M] = B
    for i in range(1, k):
        rsl_p, rsl = rsl, slice(i * N, (i + 1) * N)
        Lambda[rsl, :M] = dot(A, Lambda[rsl_p, :M])
        Lambda[rsl, M : (i + 1) * M] = Lambda[rsl_p, : i * M]

    # Build A0 matrix
    N = M = 6
    A0 = np.zeros((6, 6*k)) # 6 x 6k
    rsl = slice(0, M)
    A0[:, rsl] = A.T
    for i in range(1, k):
        rsl_p, rsl = rsl, slice(i * M, (i + 1) * M)
        A0[:, rsl] = dot(A, A0[:, rsl_p].T).T
    A0 = A0.T

    # Build acc matrix, assuming accc is constant over horizon
    U = u
    for i in range(1, k):
        U = vstack((U, u))

    # Computing predicted trajectory
    # X_pred = dot(Lambda, U)
    X_pred = dot(A0, x) + dot(Lambda, U)

    return X_pred
    # return None

def solve():
    M = array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
    P = dot(M.T, M)  # quick way to build a symmetric matrix
    q = dot(array([3., 2., 3.]), M).reshape((3,))
    G = array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
    h = array([3., 2., -2.]).reshape((3,))
    A = array([1., 1., 1.])
    b = array([1.])

    x = solve_qp(P, q, G, h, A, b)
    print("QP solution: x = {}".format(x))

if __name__ == '__main__':
    x = algo()
    
    print "Final pos: {}".format(x[0:2, -1])
    plot_traj(x, h)

    # get_states(0,0)
    # solve()