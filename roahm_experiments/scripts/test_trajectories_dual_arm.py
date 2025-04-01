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

from roahm_trajectories import TrajectoryMacros

import trajectory_helper

class TestNode(Node):
    def __init__(self):
        super().__init__("trajectory_test_node")
        # trajectory publisher
        self.traj_pub_1 = self.create_publisher(
            TrajectoryMsg, "/trajectory_1", 10)
        self.traj_pub_2 = self.create_publisher(
            TrajectoryMsg, "/trajectory_2", 10)
        
        self.traj_msg: TrajectoryMsg = TrajectoryMsg()

        # joint measurement
        self.joint_info_sub_1 = self.create_subscription(
            KortexMeasurements, "/joint_info_1", self._joint_info_1_callback, 10)
        self.joint_info_sub_2 = self.create_subscription(
            KortexMeasurements, "/joint_info_2", self._joint_info_2_callback, 10)

        self.create_timer(3.0, self._timer_callback)

    def _joint_info_1_callback(self, msg) -> None:
        """
        callback function for joint_info_1
        """
        self.robot_1_q_current = np.array(msg.pos)
        self.robot_1_qd_current = np.array(msg.vel)
        
    def _joint_info_2_callback(self, msg) -> None:
        """
        callback function for joint_info_2
        """
        self.robot_2_q_current = np.array(msg.pos)
        self.robot_2_qd_current = np.array(msg.vel)

    def publish_test_trajectory(self):
        """
        Publish a test trajectory periodically
        """
        logging.info("Publishing a test trajectory for each robot")
        print("Publishing a test trajectory for each robot")
        
        current_time = time.time()
        
        # trajectory for robot 1
        self.traj_msg_1 = trajectory_helper.formulate_armour_trajectory_message(
            current_time + 0.05, 
            2.0, 
            2.0, 
            self.robot_1_q_current, 
            np.zeros(7), 
            np.zeros(7), 
            self.robot_1_q_current + 0.1,
            True)

        self.traj_pub_1.publish(self.traj_msg_1)
        
        # trajectory for robot 2
        self.traj_msg_2 = trajectory_helper.formulate_armour_trajectory_message(
            current_time + 0.05, 
            2.0, 
            2.0, 
            self.robot_2_q_current, 
            np.zeros(7), 
            np.zeros(7), 
            self.robot_2_q_current + 0.1,
            True)

        self.traj_pub_2.publish(self.traj_msg_2)

        # print everything for debugging
        print(current_time)
        print(time.time())
        print(self.traj_msg.traj_data[:4*7])
        print(self.traj_msg.start_time)
        print(self.traj_msg.trajectory_duration)
        print(self.traj_msg.duration)
        print(self.traj_msg.trajectory_type)
        print(self.traj_msg.is_gripper_open)
        print(self.traj_msg.reset)
        print('============================================')

    def _timer_callback(self) -> None:
        self.publish_test_trajectory()

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()