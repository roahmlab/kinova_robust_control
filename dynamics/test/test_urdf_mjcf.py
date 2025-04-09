#!/usr/bin/env python3

import dynamics_py
import numpy as np

mjcf_solver = dynamics_py.SRNEA("models/mjcf/kinova3/gen3_nofa.mjcf")
urdf_solver = dynamics_py.SRNEA("models/urdf/gen3.urdf")
q = np.random.rand(7)
print("mjcf: ")
print(mjcf_solver.solve(q, q, q, q))
print("urdf: ")
print(urdf_solver.solve(q, q, q, q))
