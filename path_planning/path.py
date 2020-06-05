#!/usr/bin/env python

"""Script to generate trajectories of multiple agents

Etapes:
    1 - Trajectoire pour un agent
    2 - Trajectoire pour plus d'un agent
    3 - Ajouter des obstacles?

TODO:
    *
"""
from numpy import array, dot
from qpsolvers import solve_qp

# Test pour une trajectoire, sans collision
pos_initial = array([1., 1., 1.])
pose_final = array([2., 2., 2.])

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
    solve()