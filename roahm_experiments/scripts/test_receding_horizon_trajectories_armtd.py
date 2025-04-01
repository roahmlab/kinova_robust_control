#!/usr/bin/env python3

import numpy as np
from enum import Enum, auto
from typing import Optional
from collections import deque
import logging
import time

import rclpy
from rclpy.node import Node
from roahm_msgs.msg import KortexMeasurements, TrajectoryMsg

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

        self.create_timer(0.6, self._timer_callback)

        self.q_start = -np.ones(7)
        self.increment = 0.35 * np.ones(7)

        self.command_id = 0
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
        current_time = time.time()

        if idx == 0:
            logging.info("Initializing to q_start first")

            self.time_start = current_time + 0.025
            self.q_goal = self.q_start
            self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                self.time_start, 
                8.0, 
                8.0, 
                self.q_current, 
                np.zeros(7), 
                np.zeros(7), 
                self.q_goal)
        elif idx == 1:
            logging.info("First trajectory to move forward")

            self.time_start = self.time_start + 8.0
            self.traj_msg = trajectory_helper.formulate_armtd_trajectory_message(
                self.time_start, 
                1.0, 
                0.5, 
                self.q_start, 
                np.zeros(7), 
                0.4 * np.ones(7),
                0.5)
        elif idx <= 3:
            logging.info("Continuous trajectory to move forward in receding horizon")

            self.time_start = self.time_start + 0.5
            self.traj_msg = trajectory_helper.formulate_armtd_trajectory_message_continuous_with_previous(
                self.time_start, 
                1.0, 
                0.5, 
                0.1 * idx * np.ones(7),
                0.5,
                self.traj_msg)
        elif idx == 4:
            logging.info("Send an invalid trajectory and the robot should still follow the previous trajectory")

            self.time_start = self.time_start - 1000.0 # invalid start time
            self.traj_msg = trajectory_helper.formulate_armtd_trajectory_message(
                self.time_start, 
                1.0, 
                0.5, 
                self.q_start - 1000.0, 
                np.zeros(7), 
                0.4 * np.ones(7),
                0.5) # invalid start pos/vel/acc since not smooth
        else:
            logging.info("Test ended")
            raise Exception("Test ended")
        
        self.traj_pub.publish(self.traj_msg)

    def _timer_callback(self) -> None:
        self.publish_test_trajectory(self.command_id)
        self.command_id += 1

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()