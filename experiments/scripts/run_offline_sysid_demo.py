#!/usr/bin/env python3

import numpy as np
import queue
import time
import os
import pinocchio as pin
import pickle as pkl
import scipy.io as sio
from scipy.signal import butter, filtfilt

import rclpy
from rclpy.node import Node
from customized_msgs.msg import KortexMeasurements, TrajectoryMsg

from trajectories import TrajectoryMacros
import trajectories_py
import trajectory_helper

from pinocchio_ik_solver import pinocchioInverseKinematicsSolver

import sys
sys.path.append("/workspaces/OnlineSafeSysID/src/RAPTOR/build/lib")
import end_effector_sysid_momentum_nanobind

urdf_filename = "/workspaces/OnlineSafeSysID/models/urdf/gen3_2f85_fixed.urdf"
config_filename = "/workspaces/OnlineSafeSysID/src/RAPTOR/Examples/Kinova/Armour/KinovaWithGripperConservativePayloadInfo.yaml"

friction_parameters = np.array([
    0.0784056716873517, 0.3555204729200747, 0.3952344013937682, 0.4534811633402858, 0.09301031196704765, 0.116913366972135, 0.1995427228470504, 
    11.31528137441497, 13.08049291337487, 10.84215579520015, 11.59371952140681, 9.804089812006776, 10.62301568718394, 9.610056515230077, 
    7.485353847104333, 7.320681749988109, 7.846123679930462, 7.528515178510875, 7.42767630853946, 7.380517931078489, 7.7248481008135, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
])

trajectory_compute = trajectories_py.TrajectoryPybindWrapper()

class TestNode(Node):
    def __init__(self):
        super().__init__("online_sysid_demo_obstacles")

        # initialize path
        if not os.path.exists('results'):
            os.makedirs('results')
        
        # trajectory publisher
        self.traj_pub = self.create_publisher(
            TrajectoryMsg, "/trajectory", 10)

        # joint measurement
        self.joint_info_sub = self.create_subscription(
            KortexMeasurements, "/joint_info", self._joint_info_callback, 10)
        
        # robot states
        self.q_initial = np.zeros(7) # predefined in config.yaml
        self.q_current = np.zeros(7)
        self.qd_current = np.zeros(7)
        
        # initial values for the end effector inertial parameters for controller
        # where the robot hasn't picked up anything yet
        self.reset_endeffector_inertial()
        
        # inverse kinematics planner
        self.ik_solver = pinocchioInverseKinematicsSolver()
        
        self.approach_q1 = None
        self.approach_q2 = None
        self.place_down_start = None
        self.place_q2 = None
        
        # gripper state true: open, false: close
        self.gripper_state = True
        
        self.move_start = np.array([-1.16396061,  0.34987045, -3.68094828, -1.75466411,  0.19968747, -1.09177839, -0.19841415])

        # # Experiment (a): 0.25 m
        # self.place_down_start = np.array([-0.50296838, -0.15090219, -2.74337607, -1.43287251, 0.06063413, -1.8474959, 1.47855803]) 
        # self.q_collision_avoid = np.array([-0.8979718, -0.20561541, -3.32970829, -1.59927577, -0.00685254, -1.85913974, 0.64003912])
        # self.q_d_collision_avoid = np.array([0.61727642, -0.51122344, 0.9066071, 0.34166603, -0.10671642, -0.75006636, 1.5709715])
        # self.q_dd_collision_avoid = np.array([4.24540552e-01, 1.87500175e+00, 7.30927099e-01, -3.32258207e-01, 8.73312941e-01, 2.19965295e+00, 2.50637011e-04])

        # Experiment (b): 0.50 m
        self.place_down_start = np.array([-0.02500949, 0.55020233, -3.05694277, -0.54403656, -0.04979576, -2.04844549, 1.59508374])
        self.q_collision_avoid = np.array([-0.66290119, -0.09099118, -3.48809793, -0.79306897, 0.04506539, -1.45191409, 0.6983348])
        self.q_d_collision_avoid = np.array([1.27386205, 0.13955803, 0.77060326, 1.17729474, -0.10645826, -0.88696009, 1.68018552])
        self.q_dd_collision_avoid = np.array([5.80625060e-01, 3.11209306e+00, 8.56248533e-01, -2.04409340e+00, 2.85996558e-01, -6.84167580e-01, -9.24945438e-11])

        # # Experiment (c)
        # self.place_down_start = np.array([ np.pi/3,   1.0472,    -np.pi,      -2.0944,      0.,         np.pi/2,    0.])
        # # self.q_collision_avoid = np.array([2.44640777e-01, 1.95058783e-02, -3.16184098e+00, -1.45400537e+00, 9.98386482e-02, 2.39701503e-01, -9.91833553e-02])
        # # self.q_d_collision_avoid = np.array([1.05981204e+00, 2.12905372e-04, 4.82268320e-01, -7.64854695e-02, -6.21949412e-02, 8.29019883e-01, 6.17720723e-02])
        # # self.q_dd_collision_avoid = np.array([7.92360579e-01, -1.41262895e+00, 6.18563901e-01, 1.07268462e+00, -1.29638185e-04, 2.14981724e-04, 3.72016730e-05])
        # self.q_collision_avoid = np.array([0.36539912, 0.06848841, -3.15121153, -1.37862696, 0.09308439, 0.26049116, -0.09729588])
        # self.q_d_collision_avoid = np.array([1.47356693, 0.31359937, 0.41831043, -0.12839833, -0.11896097, 1.60853272, 0.11975499])
        # self.q_dd_collision_avoid = np.array([-1.16611558, 1.73848655, -0.72475227, -1.51221281, 0.01694156, -0.05258959, -0.00479019])

        self.move_duration = 1.0 # Experiment (a), (b)
        # self.move_duration = 1.5 # Experiment (c)
        self.open_or_close_gripper_duration = 2.0
        
        # system identification solver
        self.forward_integration_horizon = 400
        
        self.sysid_solver = end_effector_sysid_momentum_nanobind.EndEffectorIdentificationMomentumPybindWrapper(
            urdf_filename, friction_parameters, self.forward_integration_horizon, 'Nanosecond', True)
        
        self.sysid_solver.set_ipopt_parameters(
            1e-10,          # tol
            0.8,           # max_wall_time
            0,              # print_level
            100,            # max_iter
            "adaptive",     # mu_strategy
            "ma86",         # linear_solver
            False           # gradient_check
        )
        
        # initial values for the end effector inertial parameters for planning, which is super conservative
        # read by change_endeffector_inertial
        # write by endeffector_identification
        self.theta_lb = np.array([1.2, -0.4, -0.4, -1.0, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2])
        self.theta_ub = np.array([5.2, 0.4, 0.4, -0.1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        self.theta_solution = 0.5 * (self.theta_lb + self.theta_ub)
        self.theta_uncertainty = 0.5 * (self.theta_ub - self.theta_lb)
        
        # exciting trajectory settings
        self.exciting_duration = 3.0
        
        # dumbbell tasks
        self.dumbbell_order = ['4lb', '5lb', '6lb', '7lb', '8lb']
        # self.dumbbell_order = ['8lb']
        
        # predefined pick up and place down positions in the arm frame
        # Experiment (a), (b), (c)
        self.pick_pos = {
            '4lb': np.array([-0.17, 0.4, 0.04]),
            '5lb': np.array([-0.085, 0.4, 0.045]),
            '6lb': np.array([0.0, 0.4, 0.045]),
            '7lb': np.array([0.09, 0.4, 0.045]),
            '8lb': np.array([0.185, 0.4, 0.045])
        }
        # # Experiment (a)
        # self.place_pos = {
        #     '4lb': np.array([0.25, 0.0, 0.070]),
        #     '5lb': np.array([0.25, 0.0, 0.145]),
        #     '6lb': np.array([0.25, 0.0, 0.220]),
        #     '7lb': np.array([0.25, 0.0, 0.295]),
        #     '8lb': np.array([0.25, 0.0, 0.370])
        # }
        # Experiment (b)
        self.place_pos = {
            '4lb': np.array([0.50, 0.0, 0.070]),
            '5lb': np.array([0.50, 0.0, 0.145]),
            '6lb': np.array([0.50, 0.0, 0.220]),
            '7lb': np.array([0.50, 0.0, 0.295]),
            '8lb': np.array([0.50, 0.0, 0.370])
        }
        
        self.offset_z_pos = np.array([0.0, 0.0, 0.12])
        self.offset_z_pos_smaller = np.array([0.0, 0.0, 0.06])
        
        self.exciting_q_targets = queue.Queue()
        q_targets_initial = pkl.load(open("q_targets_exciting_" + self.dumbbell_order[0] + ".pkl", "rb"))
        for idx, q_target in enumerate(q_targets_initial):
            self.exciting_q_targets.put(q_target)
        self.current_exciting_q_target = None
        
        # duration for the goto_near trajectory
        self.duration = 2.0
        
        # state machine
        self.SPIN_DURATION = 0.5 * self.exciting_duration
        self.dumbbell_id = 0 # index of dumbbell
        self.traj_id = 0 # index of discontinuous trajectory sent to the robot
        self.exciting_id = 0 # number of exciting trajectory sent to the robot
        self.previous_traj_end_time = 0.0 # end time of the previous trajectory
        self.trajectory_remaining_time = 0.0 # remaining time for the current trajectory
        self.identification_id = 0 # index of identification that has been done
        self.current_task_index = 0
        self.exciting_start_time = []
        self.initialization_done = False
        self.pick_up_done = False
        self.identification_done = False
        self.place_down_done = False

        self.create_timer(self.SPIN_DURATION, self._timer_callback)

    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
        self.q_current = np.array(msg.pos)
        self.qd_current = np.array(msg.vel)
    
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
            
        traj_msg = trajectory_helper.formulate_armour_trajectory_message(
            start_time,
            self.open_or_close_gripper_duration, 
            self.open_or_close_gripper_duration, 
            q_start, 
            np.zeros(7), 
            np.zeros(7), 
            q_start,
            is_gripper_open)
        
        self.gripper_state = is_gripper_open
        
        return traj_msg

    def goto_near(
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
            
        traj_msg = trajectory_helper.formulate_armour_trajectory_message(
            start_time, 
            duration, 
            duration, 
            q_start, 
            np.zeros(7), 
            np.zeros(7), 
            q_target,
            self.gripper_state)
        
        return traj_msg
        
    def change_endeffector_inertial(self):
        """
        Write the new end effector inertial parameters into file
        """
        new_endeffector_info = np.vstack((self.theta_solution, 
                                          self.theta_lb, 
                                          self.theta_ub))
        
        np.savetxt("/workspaces/OnlineSafeSysID/models/endeffector_inertial_parameters.txt", 
                    new_endeffector_info)
            
    def reset_endeffector_inertial(self):
        """
        reset the end effector inertial parameters, where the robot hasn't picked up anything yet
        """
        model = pin.buildModelFromUrdf(urdf_filename)
        theta_solution = model.inertias[-1].toDynamicParameters()
        theta_lb = np.zeros(10)
        theta_ub = np.zeros(10)
        for i in range(10):
            if theta_solution[i] < 0:
                theta_lb[i] = 1.2 * theta_solution[i]
                theta_ub[i] = 0.8 * theta_solution[i]
            else:
                theta_lb[i] = 0.8 * theta_solution[i]
                theta_ub[i] = 1.2 * theta_solution[i]
        new_endeffector_info = np.vstack((theta_solution, 
                                            theta_lb, 
                                            theta_ub))
        np.savetxt("/workspaces/OnlineSafeSysID/models/endeffector_inertial_parameters.txt", 
                    new_endeffector_info)
            
    def pick_up_loop(self):
        counting = time.time()
        
        if self.pick_up_done:
            return
        
        pick_pos = self.pick_pos[self.dumbbell_order[self.dumbbell_id]]
        
        traj_msg = None
            
        if self.current_task_index == 0:
            if self.dumbbell_id == 0:
                traj_msg = self.goto_near(self.move_start, self.previous_traj_end_time, 9.0, self.q_initial)
                self.trajectory_remaining_time += 9.0
                self.previous_traj_end_time = traj_msg.start_time + 9.0
                print("Approach above the object")
            else:
                current_time = time.time()
                traj_msg = trajectory_helper.formulate_Bezier_trajectory_message(
                    current_time + 0.05,
                    self.move_duration,
                    self.move_duration,
                    self.place_down_start,
                    np.zeros(7),
                    np.zeros(7),
                    self.q_collision_avoid,
                    -self.q_d_collision_avoid,
                    self.q_dd_collision_avoid,
                    self.gripper_state
                )
                self.trajectory_remaining_time += self.move_duration
                self.previous_traj_end_time = traj_msg.start_time + self.move_duration
                print("Avoid collision")

                self.traj_pub.publish(traj_msg)
                self.traj_id += 1
                time.sleep(0.5)

                traj_msg = trajectory_helper.formulate_Bezier_trajectory_message(
                    self.previous_traj_end_time,
                    self.move_duration,
                    self.move_duration,
                    self.q_collision_avoid,
                    -self.q_d_collision_avoid,
                    self.q_dd_collision_avoid,
                    self.move_start,
                    np.zeros(7),
                    np.zeros(7),
                    self.gripper_state
                )
                self.trajectory_remaining_time += self.move_duration
                self.previous_traj_end_time = traj_msg.start_time + self.move_duration
                print("Avoid collision 2")
        elif self.current_task_index == 1:
            self.approach_q1, if_success = self.ik_solver.solve(
                self.move_start, 
                pick_pos + self.offset_z_pos, 
                self.ik_solver.desiredRotation_z_neg)
            if not if_success:
                raise Exception("IK failed")
            print(self.dumbbell_id, "approach_q1: ", self.approach_q1)
            traj_msg = self.goto_near(self.approach_q1, self.previous_traj_end_time, None, self.move_start)
            self.trajectory_remaining_time += self.duration
            self.previous_traj_end_time = traj_msg.start_time + self.duration
            print("Approach above the object")
        elif self.current_task_index == 2:
            self.approach_q2, if_success = self.ik_solver.solve(
                self.approach_q1, 
                pick_pos, 
                self.ik_solver.desiredRotation_z_neg)
            if not if_success:
                raise Exception("IK failed")
            print(self.approach_q2)
            traj_msg = self.goto_near(self.approach_q2, self.previous_traj_end_time, None, self.approach_q1)
            self.trajectory_remaining_time += self.duration
            self.previous_traj_end_time = traj_msg.start_time + self.duration
            print("Approach the object")
        elif self.current_task_index == 3:
            traj_msg = self.open_or_close_gripper(False, self.previous_traj_end_time, self.approach_q2)
            self.trajectory_remaining_time += self.open_or_close_gripper_duration
            self.previous_traj_end_time = traj_msg.start_time + self.open_or_close_gripper_duration
            print("Close the gripper")
        elif self.current_task_index == 4:
            traj_msg = self.goto_near(self.approach_q1, self.previous_traj_end_time, 2.0, self.approach_q2)
            self.trajectory_remaining_time += 2.0
            self.previous_traj_end_time = traj_msg.start_time + 2.0
            print("Lift the object")
            
        if traj_msg is not None: 
            self.traj_pub.publish(traj_msg)
            self.traj_id += 1
            
        print("Waiting for pick up motion to finish, remaining time:", self.trajectory_remaining_time)
        
        self.trajectory_remaining_time -= self.SPIN_DURATION
        self.current_task_index += 1
        
        if self.trajectory_remaining_time < 0 and self.current_task_index > 4:
            self.current_task_index = 0
            self.pick_up_done = True
            self.identification_done = False
            self.place_down_done = False
            self.trajectory_remaining_time = 0.0
            self.change_endeffector_inertial() # unknown payload already picked up
                
        if time.time() - counting > self.SPIN_DURATION:
            raise Exception("Pick up loop is too slow")
                
    def place_down_loop(self):
        counting = time.time()
        
        if self.place_down_done:
            return
        
        place_pos = self.place_pos[self.dumbbell_order[self.dumbbell_id]]
        
        traj_msg = None
        
        if self.current_task_index <= 0:
            self.sysid_solver.add_trajectory_file('log/data_output_' + str(self.traj_id - 2) + '.txt')
            self.sysid_solver.add_trajectory_file('log/data_output_' + str(self.traj_id - 1) + '.txt')
            self.theta_solution, self.theta_uncertainty = self.sysid_solver.optimize()
        
            for i in range(10):
                new_theta_lb = self.theta_solution[i] - self.theta_uncertainty[i]
                new_theta_ub = self.theta_solution[i] + self.theta_uncertainty[i]
                if new_theta_lb > self.theta_lb[i]:
                    self.theta_lb[i] = new_theta_lb
                if new_theta_ub < self.theta_ub[i]:
                    self.theta_ub[i] = new_theta_ub
            
            sio.savemat("results/sysid_result_" + str(self.dumbbell_order[self.dumbbell_id]) + "_" + str(self.identification_id + 1) + ".mat", 
                {"theta_solution": self.theta_solution, 
                "theta_uncertainty": self.theta_uncertainty,
                "theta_lb": self.theta_lb,
                "theta_ub": self.theta_ub})
            
            # update solution so that it is within the bounds
            for i in range(10):
                if self.theta_solution[i] < self.theta_lb[i] or \
                   self.theta_solution[i] > self.theta_ub[i]:
                    self.theta_solution[i] = (self.theta_lb[i] + self.theta_ub[i]) / 2.0

            print("theta solution:\n", self.theta_solution)
            print("theta uncertainty:\n", self.theta_uncertainty)
            print("theta lb:\n", self.theta_lb)
            print("theta ub:\n", self.theta_ub)

            self.change_endeffector_inertial()

            self.reset_sysid()

            self.trajectory_remaining_time = self.SPIN_DURATION
            print("Wait until previous task is done")
        elif self.current_task_index == 1:
            current_time = time.time()
            if self.current_exciting_q_target is None:
                traj_msg = self.goto_near(self.move_start, current_time + 0.05, None, self.approach_q1)
            else:
                traj_msg = self.goto_near(self.move_start, current_time + 0.05, None, self.current_exciting_q_target)
            self.trajectory_remaining_time += self.duration
            self.previous_traj_end_time = traj_msg.start_time + self.duration
            print("Return to the exciting start position")
        elif self.current_task_index == 2:
            traj_msg = trajectory_helper.formulate_Bezier_trajectory_message(
                self.previous_traj_end_time,
                self.move_duration,
                self.move_duration,
                self.move_start,
                np.zeros(7),
                np.zeros(7),
                self.q_collision_avoid,
                self.q_d_collision_avoid,
                self.q_dd_collision_avoid,
                self.gripper_state
            )
            self.trajectory_remaining_time += self.move_duration
            self.previous_traj_end_time = traj_msg.start_time + self.move_duration
            print("Avoid collision 1")

            self.traj_pub.publish(traj_msg)
            self.traj_id += 1
            time.sleep(0.5)

            traj_msg = trajectory_helper.formulate_Bezier_trajectory_message(
                self.previous_traj_end_time,
                self.move_duration,
                self.move_duration,
                self.q_collision_avoid,
                self.q_d_collision_avoid,
                self.q_dd_collision_avoid,
                self.place_down_start,
                np.zeros(7),
                np.zeros(7),
                self.gripper_state
            )
            self.trajectory_remaining_time += self.move_duration
            self.previous_traj_end_time = traj_msg.start_time + self.move_duration
            print("Avoid collision 2")
        elif self.current_task_index == 3:
            self.place_q1, if_success = self.ik_solver.solve(
                self.place_down_start, 
                place_pos + self.offset_z_pos_smaller, 
                self.ik_solver.desiredRotation_z_neg_another)
            if not if_success:
                raise Exception("IK failed")   
            print(self.place_q1)
            traj_msg = self.goto_near(self.place_q1, self.previous_traj_end_time, None, self.place_down_start)
            self.trajectory_remaining_time += self.duration
            self.previous_traj_end_time = traj_msg.start_time + self.duration
            print("Approach above the place position")
        elif self.current_task_index == 4:
            self.place_q2, if_success = self.ik_solver.solve(
                self.place_q1, 
                place_pos, 
                self.ik_solver.desiredRotation_z_neg_another)
            if not if_success:
                raise Exception("IK failed")   
            print(self.place_q2)
            traj_msg = self.goto_near(self.place_q2, self.previous_traj_end_time, None, self.place_q1)
            self.trajectory_remaining_time += self.duration
            self.previous_traj_end_time = traj_msg.start_time + self.duration
            print("Approach the place position")
        elif self.current_task_index == 5:
            traj_msg = self.open_or_close_gripper(True, self.previous_traj_end_time, self.place_q2)
            self.trajectory_remaining_time += self.open_or_close_gripper_duration # hardcoded in open_or_close_gripper
            self.previous_traj_end_time = traj_msg.start_time + self.open_or_close_gripper_duration
            print("Release the gripper")
        elif self.current_task_index == 6:
            traj_msg = self.goto_near(self.place_q1, self.previous_traj_end_time, None, self.place_q2)
            self.trajectory_remaining_time += self.duration
            self.previous_traj_end_time = traj_msg.start_time + self.duration
            print("Move back")
        elif self.current_task_index == 7:
            traj_msg = self.goto_near(self.place_down_start, self.previous_traj_end_time, None, self.place_q1)
            self.trajectory_remaining_time += self.duration
            self.previous_traj_end_time = traj_msg.start_time + self.duration
            print("Move back 2")
                
        if traj_msg is not None: 
            self.traj_pub.publish(traj_msg)
            self.traj_id += 1
            
        print("Waiting for place down motion to finish, remaining time:", self.trajectory_remaining_time)
        
        self.trajectory_remaining_time -= self.SPIN_DURATION
        self.current_task_index += 1
        
        if self.trajectory_remaining_time < 0 and self.current_task_index > 7:
            self.current_task_index = 0
            self.pick_up_done = False
            self.identification_done = False
            self.place_down_done = True
            self.trajectory_remaining_time = 0.0
            self.dumbbell_id += 1
            self.reset_endeffector_inertial()
                    
        if time.time() - counting > self.SPIN_DURATION:
            raise Exception("Place down loop is too slow")
                        
    def reset_sysid(self):
        self.exciting_id = 0
        if self.dumbbell_id < len(self.dumbbell_order) - 1:
            self.exciting_q_targets = queue.Queue()
            q_targets_initial = pkl.load(open("q_targets_exciting_" + self.dumbbell_order[self.dumbbell_id + 1] + ".pkl", "rb"))
            for idx, q_target in enumerate(q_targets_initial):
                self.exciting_q_targets.put(q_target)
        self.exciting_start_time = []
        self.sysid_solver.reset()
        self.identification_id = 0
        self.theta_lb = np.array([1.2, -0.4, -0.4, -1.0, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2])
        self.theta_ub = np.array([5.2, 0.4, 0.4, -0.1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        self.theta_solution = 0.5 * (self.theta_lb + self.theta_ub)
        self.theta_uncertainty = 0.5 * (self.theta_ub - self.theta_lb)

    def plan(self):
        counting = time.time()
        
        # something went wrong with planning, so no new target received
        # terminate the sysid process now
        if self.exciting_q_targets.empty():
            self.current_task_index = 0
            self.identification_done = True
            
            # reset the identification process
            print("Finish sending exciting trajectories!")
            return
        
        # send the current exciting trajectory to the robot
        self.current_exciting_q_target = self.exciting_q_targets.get()
        
        if self.current_task_index == 0: # the robot is stopping right now, send the first exciting trajectory
            current_time = time.time()
            self.exciting_start_time = current_time + 0.05
            traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                self.exciting_start_time, 
                self.exciting_duration, 
                0.5 * self.exciting_duration, 
                self.approach_q1,
                np.zeros(7),
                np.zeros(7),
                self.current_exciting_q_target,
                self.gripper_state)
        else: # the robot is still moving, so connect with the previous trajectory
            traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                self.exciting_start_time, 
                self.exciting_duration, 
                0.5 * self.exciting_duration, 
                self.exciting_q0,
                self.exciting_qd0,
                self.exciting_qdd0,
                self.current_exciting_q_target,
                self.gripper_state)
            
        self.traj_pub.publish(traj_msg)

        if self.exciting_id > 1:
            self.sysid_solver.add_trajectory_file('log/data_output_' + str(self.traj_id - 2) + '.txt')
            
        self.traj_id += 1
        self.exciting_id += 1
        
        # update initial conditions for the next trajectory
        self.exciting_start_time += 0.5 * self.exciting_duration
        trajectory_compute.setup(
            traj_msg.start_time, 
            self.exciting_duration, 
            self.exciting_duration, 
            7, 
            TrajectoryMacros.ARMOUR_TRAJ, 
            traj_msg.traj_data)
        self.exciting_q0, self.exciting_qd0, self.exciting_qdd0 = trajectory_compute.compute(self.exciting_start_time)
             
        self.current_task_index += 1
        
        if time.time() - counting > self.SPIN_DURATION:
            raise Exception("Planning and identification together took too long")
                    
    def _timer_callback(self) -> None:
        if not self.initialization_done:
            # send a dummy trajectory to skip enabling torque control
            current_time = time.time()
            traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                current_time + 0.05, 
                2.0, 
                2.0, 
                self.q_initial, 
                np.zeros(7), 
                np.zeros(7), 
                self.q_initial,
                self.gripper_state)
            self.trajectory_remaining_time += 2.0
            self.previous_traj_end_time = traj_msg.start_time + 2.0
            print("Enabling torque control")

            self.traj_pub.publish(traj_msg)
            self.traj_id += 1
            self.initialization_done = True
        elif not self.pick_up_done:
            self.pick_up_loop()
            print(self.pick_up_done, ' pick up', self.traj_id, self.current_task_index)
        elif not self.identification_done:
            self.plan()
            print(self.identification_done, ' exciting trajectory', self.traj_id, self.current_task_index)
        else:
            self.place_down_loop()
            print(self.place_down_done, ' place down', self.traj_id, self.current_task_index)
            
        if self.dumbbell_id >= len(self.dumbbell_order):
            raise Exception("All tasks finished")

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()