#!/usr/bin/env python3

import numpy as np

# define constant trajectory macros
TRAJECTORY_DATA_SIZE = 100

ARMTD_TRAJ = 0x3001
ARMOUR_TRAJ = 0x3002
FOURIER_TRAJ = 0x3003
FIFTH_ORDER_BEZIER_TRAJ = 0x3004

FOURIER_DEGREE = 5