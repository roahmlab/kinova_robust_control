#!/usr/bin/env python3

import sys
from roahm_experiments.services import KortexSrvs
import rclpy

rclpy.init()
kortex_srv = KortexSrvs()

kortex_srv.goto_name("Zero")
kortex_srv.reset_gripper()

kortex_srv.goto_name("Home")
kortex_srv.close_gripper()


