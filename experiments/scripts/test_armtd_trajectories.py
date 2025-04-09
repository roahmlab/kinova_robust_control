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

        self.create_timer(1.5, self._timer_callback)

    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
        self.q_current = np.array(msg.pos)
        self.qd_current = np.array(msg.vel)

    def publish_test_trajectory(self):
        """
        Publish a test trajectory periodically
        """
        logging.info("Publishing a test trajectory")
        print("Publishing a test trajectory")

        self.traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
        for i in range(7):
            self.traj_msg.traj_data[3 * i + 0] = self.q_current[i]
            self.traj_msg.traj_data[3 * i + 1] = 0.0
            self.traj_msg.traj_data[3 * i + 2] = -0.4
        self.traj_msg.traj_data[3 * 7] = 0.5
        
        current_time = time.time()
        self.traj_msg.start_time = current_time + 0.05
        self.traj_msg.trajectory_duration = 1.0
        self.traj_msg.duration = 1.0
        
        self.traj_msg.dof = 7
        self.traj_msg.trajectory_type = TrajectoryMacros.ARMTD_TRAJ

        self.traj_msg.is_gripper_open = False
        self.traj_msg.reset = False

        # publish
        self.traj_pub.publish(self.traj_msg)

        # print everything for debugging
        print(current_time)
        print(time.time())
        print(self.traj_msg.traj_data[:3*7+1])
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