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

import roahm_trajectories_py
from roahm_trajectories import TrajectoryMacros

traj_data_path = 'src/kinova_control/roahm_experiments/exciting_trajectories/exciting-solution-1.csv'
BEZIER_CURVE_DURATION = 5.0

class TestNode(Node):
    def __init__(self):
        super().__init__("trajectory_test_node")
        self.dof = 7

        # trajectory publisher
        self.traj_pub = self.create_publisher(
            TrajectoryMsg, "/trajectory", 10)
        self.traj_msg: TrajectoryMsg = TrajectoryMsg()

        # joint measurement
        self.joint_info_sub = self.create_subscription(
            KortexMeasurements, "/joint_info", self._joint_info_callback, 10)
        
        self.exciting_traj_data = np.loadtxt(traj_data_path, delimiter=' ')
        if len(self.exciting_traj_data) != (2 * TrajectoryMacros.FOURIER_DEGREE + 3) * self.dof + 1:
            logging.error("Invalid trajectory data")
            return ValueError("Invalid trajectory data")

        print(self.exciting_traj_data)

        self.traj_py = roahm_trajectories_py.TrajectoryPybindWrapper()
        self.traj_py.setup(0.0, 10.0, 10.0, self.dof, TrajectoryMacros.FOURIER_TRAJ, self.exciting_traj_data)
        self.q_excit_0, self.qd_excit_0, self.qdd_excit_0 = self.traj_py.compute(0.0)

        print(self.q_excit_0)
        print(self.qd_excit_0)
        print(self.qdd_excit_0)

        self.traj_id = 0
        self.first_traj_start_time = 0.0

        self.create_timer(1.0, self._timer_callback)

    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
        self.q_current = np.array(msg.pos)
        self.qd_current = np.array(msg.vel)

    def publish_test_trajectory(self):
        """
        Publish two trajectories
        """
        # if self.traj_id == 0:
        #     logging.info("Publishing a 5th order Bezier curve trajectory to reach the initial configuration")

        #     self.traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
        #     for i in range(self.dof):
        #         self.traj_msg.traj_data[6 * i + 0] = self.q_current[i]
        #         self.traj_msg.traj_data[6 * i + 1] = self.qd_current[i]
        #         self.traj_msg.traj_data[6 * i + 2] = 0.0
        #         self.traj_msg.traj_data[6 * i + 3] = self.q_excit_0[i]
        #         self.traj_msg.traj_data[6 * i + 4] = self.qd_excit_0[i]
        #         self.traj_msg.traj_data[6 * i + 5] = self.qdd_excit_0[i]
            
        #     current_time = time.time()
        #     self.traj_msg.start_time = current_time + 0.05
        #     self.first_traj_start_time = self.traj_msg.start_time
        #     self.traj_msg.trajectory_duration = BEZIER_CURVE_DURATION
        #     self.traj_msg.duration = BEZIER_CURVE_DURATION
            
        #     self.traj_msg.dof = self.dof
        #     self.traj_msg.trajectory_type = TrajectoryMacros.FIFTH_ORDER_BEZIER_TRAJ

        #     self.traj_msg.is_gripper_open = False
        #     self.traj_msg.reset = False

        #     self.traj_pub.publish(self.traj_msg)
        # elif self.traj_id == 1:
        logging.info("Publishing a Fourier series trajectory as exciting trajectory")

        self.traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
        self.traj_msg.traj_data[:len(self.exciting_traj_data)] = self.exciting_traj_data
        
        current_time = time.time()
        # self.traj_msg.start_time = self.first_traj_start_time + BEZIER_CURVE_DURATION
        self.traj_msg.start_time = current_time + 1.0
        self.traj_msg.trajectory_duration = 10.0
        self.traj_msg.duration = 10.0
        
        self.traj_msg.dof = self.dof
        self.traj_msg.trajectory_type = TrajectoryMacros.FOURIER_TRAJ

        self.traj_msg.is_gripper_open = False
        self.traj_msg.reset = False

        self.traj_pub.publish(self.traj_msg)
        # else:
        #     logging.info("Finished all trajectories")

        # self.traj_id += 1
        raise NotImplementedError("Finish the trajectory")

    def _timer_callback(self) -> None:
        self.publish_test_trajectory()

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()