#!/usr/bin/env python3

import numpy as np
from enum import Enum, auto
from typing import Optional
from collections import deque
import logging
import time

import rclpy
from rclpy.node import Node
from customized_msgs.msg import KortexMeasurements, TrajectoryMsg

from trajectories import TrajectoryMacros

import trajectory_helper

class TestNode(Node):
    def __init__(self):
        super().__init__("trajectory_test_node")
        # trajectory publisher
        self.traj_pub = self.create_publisher(
            TrajectoryMsg, "/trajectory", 10)
        self.traj_msg: TrajectoryMsg = TrajectoryMsg()

        # joint measurement
        self.joint_info_sub = self.create_subscription(
            KortexMeasurements, "/joint_info", self._joint_info_callback, 10)
        
        self.PERIOD = 3.0 # seconds
        self.q_step = 0.1 # rad

        self.create_timer(
            2.0, # send 3.0 seconds trajectory every 2.0 seconds, note that these trajectories are continuous in time
            self._timer_callback)

        self.command_id = 0
        self.q_start = []
        self.time_start = []

    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
        self.q_current = np.array(msg.pos)
        self.qd_current = np.array(msg.vel)

    def publish_test_trajectory(self, idx: int = 0):
        """
        Publish a test trajectory periodically
        """
        logging.info("Publishing a test trajectory")

        current_time = time.time()

        if idx == 0:
            self.q_start = self.q_current
            self.time_start = current_time + 0.025
        else:
            self.q_start = self.q_start + self.q_step
            self.time_start = self.time_start + self.PERIOD

        # self.traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
        # for i in range(7):
        #     self.traj_msg.traj_data[4 * i + 0] = self.q_start[i]
        #     self.traj_msg.traj_data[4 * i + 1] = 0.0
        #     self.traj_msg.traj_data[4 * i + 2] = 0.0
        #     self.traj_msg.traj_data[4 * i + 3] = self.q_start[i] + 0.1

        # self.traj_msg.start_time = self.time_start
        # self.traj_msg.trajectory_duration = self.PERIOD
        # self.traj_msg.duration = self.PERIOD
        
        # self.traj_msg.dof = 7
        # self.traj_msg.trajectory_type = TrajectoryMacros.ARMOUR_TRAJ

        # self.traj_msg.is_gripper_open = False
        # self.traj_msg.reset = False

        # we can now use the helper function to formulate the trajectory message
        self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
            self.time_start, # starting time of the trajectory
            self.PERIOD, # duration of the trajectory
            self.PERIOD, # duration to replay the trajectory
            self.q_start, # starting position
            np.zeros(7), # starting velocity
            np.zeros(7), # starting acceleration
            self.q_start + self.q_step # ending position (0.1 radian forward for each joint)
        )

        # publish
        self.traj_pub.publish(self.traj_msg)

        print(self.traj_msg)

    def _timer_callback(self) -> None:
        self.publish_test_trajectory(self.command_id)
        self.command_id += 1

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()