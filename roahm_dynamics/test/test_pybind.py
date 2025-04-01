#!/usr/bin/env python3

import roahm_dynamics_py
import numpy as np

solver = roahm_dynamics_py.SRNEA("models/mjcf/kinova3/gen3.mjcf")
q = np.random.rand(7)
print("rnea: ")
print(solver.solve(q, q, q, q))
print("mass matrix: ")
print(solver.mass_mat(q))
solver.bias(0.03)
print("mass matrix after bias: ")
print(solver.mass_mat(q))

# model with 3% variance
int_solver = roahm_dynamics_py.INT_RNEA("models/mjcf/kinova3/gen3.mjcf", 0.03)
print("interval rnea: ")
print(int_solver.solve(q, q, q, q))
print("interval mass matrix")
print(int_solver.mass_mat(q))
