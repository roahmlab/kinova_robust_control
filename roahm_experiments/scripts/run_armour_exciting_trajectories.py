#!/usr/bin/env python3

import numpy as np
from enum import Enum, auto
from typing import Optional
from collections import deque
import logging
import time
import pinocchio as pin
import pickle as pkl
import sys
import scipy.io as sio
from scipy.signal import butter, filtfilt

import rclpy
from rclpy.node import Node
from roahm_msgs.msg import KortexMeasurements, TrajectoryMsg

from roahm_trajectories import TrajectoryMacros
import roahm_trajectories_py
import trajectory_helper

sys.path.append("/workspaces/kinova_control_docker/src/RAPTOR/build/lib")
import safe_payload_exciting_nanobind
import end_effector_sysid_nanobind
import KinovaIKMotion_nanobind
import KinovaHLP_nanobind

urdf_filename = "/workspaces/kinova_control_docker/models/urdf/gen3_2f85_dumbbell_2lb.urdf"
# urdf_filename = "/workspaces/kinova_control_docker/models/urdf/gen3_2f85_fixed.urdf"
config_filename = "/workspaces/kinova_control_docker/src/RAPTOR/Examples/Kinova/Armour/KinovaWithGripperConservativePayloadInfo.yaml"

# obstacle information (xyz, rpy, size)
obstacles = np.array([[0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 0.01], # ground
                      [0.53, 0.49, 0.56, 0.0, 0.0, 0.0, 2.0, 0.08, 1.12], # back wall
                      [-0.39, -0.82, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # camera bar near the control
                      [-0.39, 0.42, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # second camera bar
                      [0.7, 0.0, 1.12, 0.0, 0.0, 0.0, 2.0, 2.0, 0.05] # ceiling
                     ])

firction_parameters = np.array([
    0.0784056716873517, 0.3555204729200747, 0.3952344013937682, 0.4534811633402858, 0.09301031196704765, 0.116913366972135, 0.1995427228470504, 
    11.31528137441497, 13.08049291337487, 10.84215579520015, 11.59371952140681, 9.804089812006776, 10.62301568718394, 9.610056515230077, 
    7.485353847104333, 7.320681749988109, 7.846123679930462, 7.528515178510875, 7.42767630853946, 7.380517931078489, 7.7248481008135, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
])

trajectory_compute = roahm_trajectories_py.TrajectoryPybindWrapper()

class TestNode(Node):
    def __init__(self):
        super().__init__("armour_exciting_trajectories_node")
        # trajectory publisher
        self.traj_pub = self.create_publisher(
            TrajectoryMsg, "/trajectory", 10)
        self.traj_msg: TrajectoryMsg = TrajectoryMsg()

        # joint measurement
        self.joint_info_sub = self.create_subscription(
            KortexMeasurements, "/joint_info", self._joint_info_callback, 10)
        
        # robot states
        self.q_current = np.zeros(7)
        self.qd_current = np.zeros(7)
        self.q_target = np.zeros(7)
        
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
        
        # RRT planner
        self.rrtplanner = KinovaHLP_nanobind.WaypointPlanningPybindWrapper(urdf_filename)
        self.rrt_collision_buffer = 0.0 # m
        self.include_gripper_or_not = True
        self.rrtplanner.set_obstacles(obstacles, self.rrt_collision_buffer)
        self.current_waypoint_index = 0
        
        # gripper state true: open, false: close
        self.gripper_state = True
        
        # exciting trajectory planner
        self.planner = safe_payload_exciting_nanobind.SafePayloadExcitingPybindWrapper(
            urdf_filename, config_filename, True)
        
        # exciting trajectory settings
        self.exciting_duration = 3.0

        # set up local planner
        self.planner.set_obstacles(obstacles)
        self.planner.set_ipopt_parameters(
            1e-6,          # tol
            1e-6,          # constr_viol_tol
            10.0,          # obj_scaling_factor
            1.5,           # max_wall_time
            0,             # print_level
            "adaptive",    # mu_strategy
            "ma57",        # linear_solver
            False          # gradient_check
        )
        
        self.q_targets = pkl.load(open("q_targets_initial.pkl", "rb"))
        print(self.q_targets)
        
        # system identification solver
        self.forward_integration_horizon = 200
        self.cutoff_frequency = 20
        
        self.sysid_solver = end_effector_sysid_nanobind.EndEffectorIdentificationPybindWrapper(
            urdf_filename, firction_parameters, self.forward_integration_horizon, 'Nanosecond', True)
        
        self.sysid_solver.set_ipopt_parameters(
            1e-10,          # tol
            2.0,            # max_wall_time
            0,              # print_level
            200,            # max_iter
            "adaptive",     # mu_strategy
            "ma86",         # linear_solver
            False           # gradient_check
        )
        
        # initial values for the end effector inertial parameters, which is super conservative
        self.theta_lb = np.array([1.2, -0.1, -0.1, -0.8, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15])
        self.theta_ub = np.array([5.0, 0.1, 0.1, -0.1, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15])
        self.theta_solution = 0.5 * (self.theta_lb + self.theta_ub)
        self.theta_uncertainty = 0.5 * (self.theta_ub - self.theta_lb)
        
        sio.savemat("results/sysid_result_" + str(0) + ".mat", 
            {"theta_solution": self.theta_solution, 
             "theta_uncertainty": self.theta_uncertainty,
             "theta_lb": self.theta_lb,
             "theta_ub": self.theta_ub})
        
        self.trajectory_filenames = []
        
        # predefined pick up positions in the arm frame
        # self.approach_pos = np.array([0.51, -0.17, 0.04]) # For 2lb dumbbell
        # self.place_pos = np.array([0.51, -0.17, 0.035])
        # self.approach_pos = np.array([0.51, -0.17, 0.045]) # For 3lb dumbbell
        # self.place_pos = np.array([0.51, -0.17, 0.04])
        # self.approach_pos = np.array([0.51, -0.17, 0.05]) # For 4lb and 5lb dumbbell
        # self.place_pos = np.array([0.51, -0.17, 0.045])
        self.approach_pos = np.array([0.51, -0.17, 0.055]) # For 6lb and 7lb dumbbell
        self.place_pos = np.array([0.51, -0.17, 0.05])
        # self.place_pos = np.array([0.0, -0.3, 0.03])
        self.offset_pos = np.array([0.0, 0.0, 0.10])
        
        # RRT planning from approach to place
        # self.RRTPlan(self.approach_q, self.place_q)
        
        # duration for the goto trajectory
        self.duration = 9.0
        
        # state machine
        self.action_id = 0 # index of ros spin function (_timer_callback)
        self.traj_id = 0 # index of discontinuous trajectory sent to the robot
        self.identification_id = 0 # index of identification that has been done
        self.current_task_index = 0
        self.pick_up_done = False
        self.identification_done = False
        self.place_down_done = False

        self.create_timer(12.0, self._timer_callback)

    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
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
        new_endeffector_info = np.vstack((self.theta_solution, 
                                          self.theta_lb, 
                                          self.theta_ub))
        
        np.savetxt("/workspaces/kinova_control_docker/models/endeffector_inertial_parameters.txt", 
                   new_endeffector_info)
    
    def RRTPlan(self, start, goal):
        self.rrtplanner.set_start_goal(start, goal)
        try:
            rrtplanner_time = 2.0 # sec
            self.rrtpath = self.rrtplanner.plan(rrtplanner_time, self.include_gripper_or_not)
            print(len(self.rrtpath))
        except Exception as e:
            print(e)
            self.rrtpath = []
            
    def pick_up_loop(self):
        if not self.pick_up_done:
            if self.current_task_index == 0:
                self.approach_q1 = self.IK(self.approach_pos + self.offset_pos)
                self.goto(self.approach_q1)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                self.current_task_index += 1
                print("approach configuration:", self.approach_q1)
                print("Approach above the object")
            elif self.current_task_index == 1:
                self.approach_q2 = self.IK(self.approach_pos)
                print("approach close configuration:", self.approach_q2)
                
                current_time = time.time()
                
                start_time_1 = current_time + 0.05
                
                self.goto(self.approach_q2, start_time_1, 3.0)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.1)
                print("Approach the object")
                
                start_time_2 = start_time_1 + 3.0 + 0.05
                self.open_or_close_gripper(False, start_time_2, self.approach_q2)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.1)
                print("Close the gripper")
                
                start_time_3 = start_time_2 + 2.5 + 0.2
                self.goto(self.approach_q1, start_time_3, 3.0, self.approach_q2)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.1)
                print("Pick up the object")
                
                self.current_task_index = 0
                self.pick_up_done = True
                self.identification_done = False
                self.place_down_done = False
                
    def place_down_loop(self):
        if not self.place_down_done:
            if self.current_task_index == 0:
                self.place_q1 = self.IK(self.place_pos + self.offset_pos)
                print("place over configuration:", self.place_q1)
                self.goto(self.place_q1)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                self.current_task_index += 1
                print("Approach above the place position")
            elif self.current_task_index == 1:
                self.place_q2 = self.IK(self.place_pos)
                print("place down configuration:", self.place_q2)
                self.goto(self.place_q2)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                self.current_task_index += 1
                print("Approach the place position")
            elif self.current_task_index == 2:
                current_time = time.time()
                
                start_time_1 = current_time + 0.05
                
                self.open_or_close_gripper(True, start_time_1)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.1)
                print("Release the gripper")
                
                start_time_2 = start_time_1 + 2.5 + 0.2
                
                self.goto(self.place_q1, start_time_2, 3.0)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.1)
                print("Finish placing down")
                
                self.current_task_index = 0
                self.place_down_done = True
                self.pick_up_done = False
                    
    def plan_exciting_trajectory(self):
        logging.info("Computing a exciting trajectory")
        print("Computing a exciting trajectory")
        
        # self.current_waypoint_index += 20
        # if self.current_waypoint_index >= len(self.rrtpath):
        #     q_nominal = np.array(self.rrtpath[-1, :])
        #     self.current_waypoint_index = len(self.rrtpath)
        # else:
        #     q_nominal = np.array(self.rrtpath[self.current_waypoint_index, :])
        
        k_center = np.zeros(7)
        # k_center[-1] = q_nominal[-1] + 0.1
        # k_center[4:] = q_nominal[4:] - self.q_current[4:]
        k_range = np.zeros(7)
        k_range[:4] = np.pi/48
        k_range[4:] = np.pi/20
        
        q0 = self.q_current
        qd0 = self.qd_current
        qdd0 = np.zeros(7)
        
        start_time = 0.0
        
        # plan for 5 continuous receding horizon exciting Armour trajectories
        for i in range(5):
            self.planner.set_trajectory_parameters(
                q0, \
                qd0, \
                qdd0, \
                k_center, \
                k_range, \
                self.exciting_duration)
            
            if self.identification_id > 0:
                self.planner.set_endeffector_inertial_parameters(
                    self.theta_solution, \
                    self.theta_lb, \
                    self.theta_ub)
            
            k, ifFeasible = self.planner.optimize()
            
            if ifFeasible:
                q_target = q0 + k_center + k * k_range
            else:
                raise Exception("Trajectory not feasible!")
            
            # construct the new q_targets
            if i == 0:
                self.q_targets = q_target
            else:
                self.q_targets = np.vstack((self.q_targets, q_target))
            
            traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                start_time, 
                self.exciting_duration, 
                0.5 * self.exciting_duration, 
                q0, 
                qd0, 
                qdd0, 
                q_target)
            
            trajectory_compute.setup(start_time, 
                self.exciting_duration, 
                self.exciting_duration, 
                7, 
                TrajectoryMacros.ARMOUR_TRAJ, 
                traj_msg.traj_data)
            q0, qd0, qdd0 = trajectory_compute.compute(start_time + 0.5 * self.exciting_duration)
            
            start_time += 0.5 * self.exciting_duration
            
        print("q_targets:\n", self.q_targets)
            
    def endeffector_identification(self):
        logging.info("Performing system identification")
        print("Performing system identification")
    
        # the following trajectory should just have been executed
        data = np.loadtxt('log/torque_output_' + str(self.traj_id - 1) + '.txt')
        
        # filter velocity and torque data
        t = data[:, 0]
        q = data[:, 1:8]
        qd = data[:, 8:15]
        tau = data[:, 15:22]

        dt = np.mean(np.diff(t / 1e9))
        fs = 1 / dt

        order = 4
        normal_cutoff = 20
        b, a = butter(order, normal_cutoff, btype='low', analog=False, fs=fs)

        qd_filtered = qd
        tau_filtered = tau

        for i in range(7):
            qd_filtered[:, i] = filtfilt(b, a, qd[:, i])
            tau_filtered[:, i] = filtfilt(b, a, tau[:, i])
            
        data_filtered = np.hstack((t[:, None], q, qd_filtered, tau_filtered))
        np.savetxt('log/torque_output_' + str(self.traj_id - 1) + '_filtered.txt', data_filtered, fmt='%.12f')
        
        # add the filtered trajectory file name to the list and perform system identification
        self.trajectory_filenames.append('log/torque_output_' + str(self.traj_id - 1) + '_filtered.txt')
        self.theta_solution, self.theta_uncertainty = self.sysid_solver.optimize(self.trajectory_filenames)
        
        # update the end effector inertial parameters lower bound, upper bound
        # change the solution if it is out of the bounds
        for i in range(10):
            new_theta_lb = self.theta_solution[i] - self.theta_uncertainty[i]
            new_theta_ub = self.theta_solution[i] + self.theta_uncertainty[i]
            if new_theta_lb > self.theta_ub[i]:
                raise Exception("Infeasible solution")
            if new_theta_ub < self.theta_lb[i]:
                raise Exception("Infeasible solution")
            
            if new_theta_lb > self.theta_lb[i]:
                self.theta_lb[i] = new_theta_lb
            if new_theta_ub < self.theta_ub[i]:
                self.theta_ub[i] = new_theta_ub
        
        print("theta solution:\n", self.theta_solution)
        print("theta uncertainty:\n", self.theta_uncertainty)
        print("theta lb:\n", self.theta_lb)
        print("theta ub:\n", self.theta_ub)
        
        sio.savemat("results/sysid_result_" + str(self.identification_id + 1) + ".mat", 
            {"theta_solution": self.theta_solution, 
             "theta_uncertainty": self.theta_uncertainty,
             "theta_lb": self.theta_lb,
             "theta_ub": self.theta_ub})
        
        # update solution so that it is within the bounds
        for i in range(10):
            if self.theta_solution[i] < self.theta_lb[i] or \
               self.theta_solution[i] > self.theta_ub[i]:
                self.theta_solution[i] = (self.theta_lb[i] + self.theta_ub[i]) / 2.0
        
        self.identification_id += 1
        # self.change_endeffector_inertial()

    def plan(self):
        if self.current_task_index > 0:
            # perform system identification on the previous trajectory that has been executed
            # self.theta_solution, self.theta_lb, self.theta_ub are updated here
            counting = time.time()
            self.endeffector_identification()
            print("Identification took: ", time.time() - counting, " seconds")
            
        # change the end effector inertial parameters here
        # since the robot should stop right now
        # and the end effector inertial parameters are updated if at least one identification has finished
        self.change_endeffector_inertial()
        
        current_time = time.time()
        start_time = current_time + 0.05
        
        # send a series of exciting trajectories where q_targets have been initialized or updated
        for i, q_target in enumerate(self.q_targets):
            if i == 0:
                self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                    start_time, 
                    self.exciting_duration, 
                    0.5 * self.exciting_duration, 
                    self.q_current,
                    self.qd_current,
                    np.zeros(7),
                    q_target,
                    self.gripper_state)
            else:
                self.traj_msg = trajectory_helper.formulate_armour_trajectory_message_continuous_with_previous(
                    start_time,
                    self.exciting_duration,
                    0.5 * self.exciting_duration,
                    q_target,
                    self.traj_msg)
                
            self.traj_pub.publish(self.traj_msg)
            time.sleep(0.1)
            start_time += 0.5 * self.exciting_duration
        
        if self.current_task_index >= 4:
            self.current_task_index = 0
            self.identification_done = True
            self.trajectory_filenames = []
            self.q_targets = []
            return
        else:
            if self.current_task_index > 0:
                # plan for the new exciting trajectories
                # self.q_targets is updated here
                counting = time.time()
                self.plan_exciting_trajectory()
                print("Planning took: ", time.time() - counting, " seconds")

        self.traj_id += 1
        self.current_task_index += 1
                    
    def _timer_callback(self) -> None:
        if not self.pick_up_done:
            self.pick_up_loop()
            print(self.action_id, self.pick_up_done, ' pick up')
        elif not self.identification_done:    
            self.plan()
            print(self.action_id, self.identification_done, ' exciting trajectory')
        else:
            self.place_down_loop()
            print(self.action_id, self.place_down_done, ' place down')
        self.action_id += 1
        
        if self.action_id >= 10:
            raise Exception("Finished")

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()