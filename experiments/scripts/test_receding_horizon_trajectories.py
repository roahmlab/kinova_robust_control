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
        
        self.receding_horizon_duration = 1.0 # seconds

        self.create_timer(
            0.4, # make sure this is less than the receding_horizon_duration
            self._timer_callback)

        # self.q_start = np.array([0.0, 0.262, 3.142, -2.269, 0.0, 0.960, 1.571]) # home position
        self.q_start = np.zeros(7) # zero position
        self.q_step = 0.1 * np.ones(7)

        self.command_id = 0
        self.q_goal = []
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
                6.0, 
                6.0, 
                self.q_current, 
                np.zeros(7), 
                np.zeros(7), 
                self.q_goal)
        elif idx == 1:
            logging.info("First trajectory to move forward")

            self.time_start = self.time_start + 6.0
            self.q_goal = self.q_goal + self.q_step
            self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                self.time_start, 
                self.receding_horizon_duration, 
                0.5 * self.receding_horizon_duration, 
                self.q_start, 
                np.zeros(7), 
                np.zeros(7), 
                self.q_goal)
        elif idx <= 5:
            logging.info("Continuous trajectory to move forward in receding horizon")

            self.time_start = self.time_start + 0.5 * self.receding_horizon_duration
            self.q_goal = self.q_goal + self.q_step
            self.traj_msg = trajectory_helper.formulate_armour_trajectory_message_continuous_with_previous(
                self.time_start, 
                self.receding_horizon_duration, 
                0.5 * self.receding_horizon_duration, 
                self.q_goal, 
                self.traj_msg)
        elif idx == 6:
            logging.info("Send an invalid trajectory, so the robot should ignore this one and still follow the previous trajectory until stopping")

            self.time_start = self.time_start - 1000.0 # invalid start time
            self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                self.time_start, 
                4.0, 
                4.0, 
                np.zeros(7), 
                np.zeros(7),
                np.zeros(7), 
                np.zeros(7)) # invalid start pos/vel/acc since not smooth
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