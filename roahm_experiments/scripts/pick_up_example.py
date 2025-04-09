#!/usr/bin/env python3

import numpy as np
from enum import Enum, auto
from typing import Optional
from collections import deque
import logging
import time
from numpy.linalg import norm, solve
from roahm_experiments.services import KortexSrvs
from pinocchio_ik_solver import pinocchioInverseKinematicsSolver
import rclpy

rclpy.init()
kortex_srv = KortexSrvs()

ik_solver = pinocchioInverseKinematicsSolver()

# served as initial guess in IK, should usually be the current position of the arm
q_init = np.array([0.0, 0.262, 3.142, -2.2692, 0.0, -0.6108, 1.571])
kortex_srv.goto(q_init.astype(np.float32))

# predefined pick up positions in the arm base frame
approach_pos = np.array([0.52, -0.1778, 0.15])
grasp_pos = np.array([0.52, -0.1778, 0.05])

# solve IK for approach and retract configurations
approach_q, if_success = ik_solver.solve(
    q_init, 
    approach_pos,
    ik_solver.desiredRotation_z_neg)
if if_success:
    print(approach_q)
    approach_q = approach_q.astype(np.float32)
else:
    raise Exception("IK failed")

grasp_q, if_success = ik_solver.solve(
    approach_q, 
    grasp_pos,
    ik_solver.desiredRotation_z_neg)
if if_success:
    print(grasp_q)
    grasp_q = grasp_q.astype(np.float32)
else:
    raise Exception("IK failed")
    
print("Start pick up")

kortex_srv.reset_gripper()
kortex_srv.goto(approach_q)

print("Finish approach")

kortex_srv.goto(grasp_q)
kortex_srv.close_gripper()

print("Finish grasp")

kortex_srv.goto(approach_q)

print("Finish retract")