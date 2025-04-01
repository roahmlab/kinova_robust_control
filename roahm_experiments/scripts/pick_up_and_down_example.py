#!/usr/bin/env python3

import numpy as np
from enum import Enum, auto
from typing import Optional
from collections import deque
import pinocchio as pin
import time
import sys
import rclpy
from rclpy.node import Node
from roahm_msgs.msg import KortexMeasurements, TrajectoryMsg

from roahm_trajectories import TrajectoryMacros
sys.path.append("/workspaces/kinova_control_docker/src/RAPTOR/build/lib")
import KinovaIKMotion_nanobind

import trajectory_helper

urdf_filename = "/workspaces/kinova_control_docker/models/urdf/gen3_2f85_dumbbell_3lb.urdf"

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
        
        # inverse kinematics planner
        self.ik_motion_solver = KinovaIKMotion_nanobind.KinovaIKMotionPybindWrapper(urdf_filename, False)
        self.ik_motion_solver.set_ipopt_parameters(
            1e-6,          # tol
            1e-8,          # constr_viol_tol
            1.0,           # obj_scaling_factor
            1.0,           # max_wall_time
            0,             # print_level
            "monotone",    # mu_strategy
            "ma57",        # linear_solver
            False          # gradient_check
        )
        
        # end effector inertial parameters after picking up
        model = pin.buildModelFromUrdf(urdf_filename)
        self.new_endeffector_inertial = model.inertias[-1].toDynamicParameters()
        print(self.new_endeffector_inertial)

        # gripper state true: open, false: close
        self.gripper_state = True

        self.duration = 6.0
        
        # predefined pick up positions in the arm frame
        # self.approach_pos = np.array([0.51, -0.17, 0.04]) # For 2lb dumbbell
        # self.place_pos = np.array([0.0, -0.5, 0.03])
        self.approach_pos = np.array([0.51, -0.17, 0.045]) # For 3lb dumbbell
        self.place_pos = np.array([0.0, -0.5, 0.04])
        self.offset_pos = np.array([0.0, 0.0, 0.12])
        
        # state machine
        self.current_task_index = 0
        self.traj_id = 0
        self.trajectory_remaining_time = 0.0
        self.SPIN_DURATION = 1.5
        self.pick_up_done = False
        self.place_down_done = False

        self.create_timer(self.SPIN_DURATION, self._timer_callback)

    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
        # self.get_logger().info(f"Received joint info))")
        self.q_current = np.array(msg.pos)
        self.qd_current = np.array(msg.vel)
        
    def IK(self, desiredTranslation):
        """
        Inverse kinematics solver
        """
        desiredTransforms = np.zeros((1, 12))
        desiredRotation = np.array([
            [1,  0,  0],   
            [0, -1,  0],   
            [0,  0, -1]
        ])
        desiredTransforms[0] = np.concatenate([desiredTranslation, desiredRotation.flatten()])
        self.ik_motion_solver.set_desired_endeffector_transforms(desiredTransforms)
        qs, if_success = self.ik_motion_solver.solve(self.q_current)
        return qs.flatten()
        
    def open_or_close_gripper(
        self,
        is_gripper_open: bool, 
        start_time: float = None,
        q_start: np.ndarray = None):
        """
        Open or close the gripper
        """
        if start_time is None:
            start_time = time.time() + 0.05
            
        if q_start is None:
            q_start = self.q_current
            
        self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
            start_time,
            2.5, 
            2.5, 
            q_start, 
            np.zeros(7), 
            np.zeros(7), 
            q_start,
            is_gripper_open)
        
        self.gripper_state = is_gripper_open

    def goto(
        self, 
        q_target: np.ndarray, 
        start_time: float = None,
        duration: float = None,
        q_start: np.ndarray = None):
        """
        Go to a target joint configuration
        """
        if start_time is None:
            start_time = time.time() + 0.05
            
        if duration is None:
            duration = self.duration
            
        if q_start is None:
            q_start = self.q_current
            
        self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
            start_time, 
            duration, 
            duration, 
            q_start, 
            np.zeros(7), 
            np.zeros(7), 
            q_target,
            self.gripper_state)
        
    def change_endeffector_inertial(self):
        """
        Write the new end effector inertial parameters into file
        """
        self.new_endeffector_inertial_lb = np.zeros(10)
        self.new_endeffector_inertial_ub = np.zeros(10)
        
        for i in range(10):
            if self.new_endeffector_inertial[i] > 0:
                self.new_endeffector_inertial_lb[i] = self.new_endeffector_inertial[i] * 0.8
                self.new_endeffector_inertial_ub[i] = self.new_endeffector_inertial[i] * 1.2
            else:
                self.new_endeffector_inertial_lb[i] = self.new_endeffector_inertial[i] * 1.2
                self.new_endeffector_inertial_ub[i] = self.new_endeffector_inertial[i] * 0.8
                
        new_endeffector_info = np.vstack((self.new_endeffector_inertial, 
                                          self.new_endeffector_inertial_lb, 
                                          self.new_endeffector_inertial_ub))
        
        np.savetxt("/workspaces/kinova_control_docker/models/endeffector_inertial_parameters.txt", 
                   new_endeffector_info)
        
    def pick_up_loop(self):
        if not self.pick_up_done:
            if self.current_task_index == 0:
                if self.trajectory_remaining_time == 0:
                    self.approach_q1 = self.IK(self.approach_pos + self.offset_pos)
                    self.goto(self.approach_q1)
                    self.traj_pub.publish(self.traj_msg)
                    self.traj_id += 1
                    self.trajectory_remaining_time = self.duration
                    print("configuration:", self.approach_q1)
                    print("Approach above the object")
                else:
                    self.trajectory_remaining_time -= self.SPIN_DURATION
                    print("Waiting for pick up - approach motion to finish, remaining time:", self.trajectory_remaining_time)
                    if self.trajectory_remaining_time <= 0:
                        self.current_task_index += 1
                        self.trajectory_remaining_time = 0.0
            elif self.current_task_index == 1:
                if self.trajectory_remaining_time == 0:
                    self.approach_q2 = self.IK(self.approach_pos)
                    
                    current_time = time.time()
                    
                    start_time_1 = current_time + 0.05
                    
                    self.goto(self.approach_q2, start_time_1, 3.0)
                    self.traj_pub.publish(self.traj_msg)
                    time.sleep(0.01)
                    print("Approach the object")
                    
                    start_time_2 = start_time_1 + 3.0
                    self.open_or_close_gripper(False, start_time_2, self.approach_q2)
                    self.traj_pub.publish(self.traj_msg)
                    time.sleep(0.01)
                    print("Close the gripper")
                    
                    start_time_3 = start_time_2 + 2.5
                    self.goto(self.approach_q1, start_time_3, 3.0, self.approach_q2)
                    self.traj_pub.publish(self.traj_msg)
                    time.sleep(0.01)
                    print("Pick up the object")
                    
                    self.change_endeffector_inertial()
                    
                    # total time for the above three trajectories is 3.0 + 2.5 + 3.0 = 8.5
                    self.trajectory_remaining_time = 8.5
                    self.traj_id += 1
                else:
                    self.trajectory_remaining_time -= self.SPIN_DURATION
                    print("Waiting for pick up - pick up motion to finish, remaining time:", self.trajectory_remaining_time)
                    if self.trajectory_remaining_time <= 0:
                        self.current_task_index = 0
                        self.pick_up_done = True
                        self.place_down_done = False
                        self.trajectory_remaining_time = 0.0
                    
    def place_down_loop(self):
        if not self.place_down_done:
            if self.current_task_index == 0:
                if self.trajectory_remaining_time == 0:
                    self.place_q1 = self.IK(self.place_pos + self.offset_pos)
                    self.goto(self.place_q1)
                    self.traj_pub.publish(self.traj_msg)
                    self.traj_id += 1
                    self.trajectory_remaining_time = self.duration
                    print("Approach above the place position")
                else:
                    self.trajectory_remaining_time -= self.SPIN_DURATION
                    print("Waiting for place down - approach motion to finish, remaining time:", self.trajectory_remaining_time)
                    if self.trajectory_remaining_time <= 0:
                        self.current_task_index += 1
                        self.trajectory_remaining_time = 0.0
            elif self.current_task_index == 1:
                if self.trajectory_remaining_time == 0:
                    self.place_q2 = self.IK(self.place_pos)
                    self.goto(self.place_q2)
                    self.traj_pub.publish(self.traj_msg)
                    self.traj_id += 1
                    self.trajectory_remaining_time = self.duration
                    print("Approach the place position")
                else:
                    self.trajectory_remaining_time -= self.SPIN_DURATION
                    print("Waiting for place down - approach motion to finish, remaining time:", self.trajectory_remaining_time)
                    if self.trajectory_remaining_time <= 0:
                        self.current_task_index += 1
                        self.trajectory_remaining_time = 0.0
            elif self.current_task_index == 2:
                if self.trajectory_remaining_time == 0:
                    current_time = time.time()
                    
                    start_time_1 = current_time + 0.05
                    
                    self.open_or_close_gripper(True, start_time_1)
                    self.traj_pub.publish(self.traj_msg)
                    time.sleep(0.01)
                    print("Release the gripper")
                    
                    start_time_2 = start_time_1 + 2.5
                    
                    self.goto(self.place_q1, start_time_2, 3.0)
                    self.traj_pub.publish(self.traj_msg)
                    time.sleep(0.01)
                    print("Finish placing down")
                    
                    # total time for the above two trajectories is 2.5 + 3.0 = 5.5
                    self.trajectory_remaining_time = 5.5
                    self.traj_id += 1
                else:
                    self.trajectory_remaining_time -= self.SPIN_DURATION
                    print("Waiting for place down - place down motion to finish, remaining time:", self.trajectory_remaining_time)
                    if self.trajectory_remaining_time <= 0:
                        self.current_task_index = 0
                        self.place_down_done = True
                        self.pick_up_done = False
                        self.trajectory_remaining_time = 0.0
                        
                        raise Exception("Finish the pick up and place down task")
                
    def _timer_callback(self) -> None:
        if not self.pick_up_done:
            self.pick_up_loop()
            print(self.pick_up_done, ' pick up')
        else:
            self.place_down_loop()
            print(self.pick_up_done, ' place down')

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()