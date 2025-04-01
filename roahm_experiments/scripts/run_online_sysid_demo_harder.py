#!/usr/bin/env python3

import numpy as np
import queue
import time
import copy
import pinocchio as pin
import pickle as pkl
import threading
import scipy.io as sio
from scipy.signal import butter, filtfilt

import rclpy
from rclpy.node import Node
from roahm_msgs.msg import KortexMeasurements, TrajectoryMsg

from roahm_trajectories import TrajectoryMacros
import roahm_trajectories_py
import trajectory_helper

import sys
sys.path.append("/workspaces/kinova_control_docker/src/RAPTOR/build/lib")
import safe_payload_exciting_nanobind
import end_effector_sysid_nanobind
import KinovaIKMotion_nanobind
import KinovaHLP_nanobind

# urdf_filename = "/workspaces/kinova_control_docker/models/urdf/gen3_2f85_dumbbell_5lb.urdf"
urdf_filename = "/workspaces/kinova_control_docker/models/urdf/gen3_2f85_fixed.urdf"
config_filename = "/workspaces/kinova_control_docker/src/RAPTOR/Examples/Kinova/Armour/KinovaWithGripperConservativePayloadInfo.yaml"

# obstacle information (xyz, rpy, size)
obstacles = np.array([[0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 0.01], # ground
                      [0.53, 0.49, 0.56, 0.0, 0.0, 0.0, 2.0, 0.08, 1.12], # back wall
                      [-0.39, -0.82, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # camera bar near the control
                      [-0.39, 0.42, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # second camera bar
                      [0.7, 0.0, 1.12, 0.0, 0.0, 0.0, 2.0, 2.0, 0.05], # ceiling
                      [0.51, 0.0, 0.05, 0.0, 0.0, 0.0, 0.3, 0.1, 0.05] # other dumbbells
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
        
        self.endeffector_inertial_lock = threading.Lock()  # Lock for thread safety
        self.exciting_initial_conditions_lock = threading.Lock()  # Lock for thread safety
        self.trajectory_data_lock = threading.Lock()  # Lock for thread safety
        
        # trajectory publisher
        self.traj_pub = self.create_publisher(
            TrajectoryMsg, "/trajectory", 10)
        self.traj_msg: TrajectoryMsg = TrajectoryMsg()

        # joint measurement
        self.joint_info_sub = self.create_subscription(
            KortexMeasurements, "/joint_info", self._joint_info_callback, 10)
        
        # robot states
        self.q_current = np.array([0.0, 0.262, 3.142, -2.2692, 0.0, -0.6108, 1.571]) # predefined in config.yaml
        self.qd_current = np.zeros(7)
        
        # this desired rotation is gripper pointing along the positive x axis
        self.desiredRotation_x_pos = np.array([
            [0,  0, 1],   
            [-1, 0, 0],   
            [0, -1, 0]
        ])
        
        self.desiredRotation_y_neg = np.array([
            [-1, 0, 0],   
            [0, 0, -1],   
            [0, -1, 0]
        ])
        
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
        
        self.approach_q1 = None
        self.approach_q2 = None
        self.place_q1 = None
        self.place_q2 = None
        self.place_q3 = None
        
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
            1.0,           # obj_scaling_factor
            0.7,           # max_wall_time
            5,             # print_level
            "monotone",    # mu_strategy
            "ma57",        # linear_solver
            False          # gradient_check
        )
        
        self.exciting_q_targets = queue.Queue()
        q_targets_initial = pkl.load(open("q_targets_initial.pkl", "rb"))
        for q_target in q_targets_initial:
            self.exciting_q_targets.put(q_target)
        print(self.exciting_q_targets)
        
        # system identification solver
        self.forward_integration_horizon = 200
        self.cutoff_frequency = 20
        
        self.sysid_solver = end_effector_sysid_nanobind.EndEffectorIdentificationPybindWrapper(
            urdf_filename, firction_parameters, self.forward_integration_horizon, 'Nanosecond', True)
        
        self.sysid_solver.set_ipopt_parameters(
            1e-10,          # tol
            0.45,            # max_wall_time
            0,              # print_level
            100,            # max_iter
            "adaptive",     # mu_strategy
            "ma86",         # linear_solver
            False           # gradient_check
        )
        
        # initial values for the end effector inertial parameters, which is super conservative
        # read by change_endeffector_inertial and plan_exciting_trajectory
        # write by endeffector_identification
        self.theta_lb = np.array([1.2, -0.1, -0.1, -0.8, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15])
        self.theta_ub = np.array([5.0, 0.1, 0.1, -0.1, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15])
        self.theta_solution = 0.5 * (self.theta_lb + self.theta_ub)
        self.theta_uncertainty = 0.5 * (self.theta_ub - self.theta_lb)
        
        sio.savemat("results/sysid_result_" + str(0) + ".mat", 
            {"theta_solution": self.theta_solution, 
             "theta_uncertainty": self.theta_uncertainty,
             "theta_lb": self.theta_lb,
             "theta_ub": self.theta_ub})
        
        # dumbbell tasks
        self.dumbbell_order = ['3lb', '4lb', '5lb', '6lb', '7lb']
        
        # predefined pick up and place down positions in the arm frame
        self.pick_pos = {
            '3lb': np.array([0.67, 0.0, 0.375 - 0.01]),
            '4lb': np.array([0.67, 0.0, 0.310 - 0.01]),
            '5lb': np.array([0.67, 0.0, 0.235 - 0.01]),
            '6lb': np.array([0.67, 0.0, 0.157 - 0.01]),
            '7lb': np.array([0.67, 0.0, 0.078 - 0.01])
        }
        self.place_pos = {
            '3lb': np.array([0.0, -0.67, 0.070 - 0.00]),
            '4lb': np.array([0.0, -0.67, 0.133 - 0.00]),
            '5lb': np.array([0.0, -0.67, 0.205 - 0.00]),
            '6lb': np.array([0.0, -0.67, 0.280 - 0.00]),
            '7lb': np.array([0.0, -0.67, 0.360 - 0.00])
        }
        self.offset_x_neg = np.array([-0.10, 0.0, 0.0])
        self.offset_y_pos = np.array([0.0, 0.10, 0.0])
        self.offset_z_pos = np.array([0.0, 0.0, 0.10])
        
        # RRT planning from approach to place
        # self.RRTPlan(self.approach_q, self.place_q)
        
        # duration for the goto_near trajectory
        self.duration = 6.0
        
        # state machine
        self.SPIN_DURATION = 0.5 * self.exciting_duration
        self.dumbbell_id = 0 # index of dumbbell
        self.traj_id = 0 # index of discontinuous trajectory sent to the robot
        self.exciting_id = 0 # number of exciting trajectory sent to the robot
        self.trajectory_remaining_time = 0.0 # remaining time for the current trajectory
        self.identification_id = 0 # index of identification that has been done
        self.current_task_index = 0
        self.exciting_start_time = []
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
        
    def IK(
        self, 
        desiredTranslation: np.ndarray, 
        desiredRotation: np.ndarray = None,
        q_start: np.ndarray = None) -> np.ndarray:
        """
        Inverse kinematics solver
        """
        desiredTransforms = np.zeros((1, 12))
        if desiredRotation is None:
            # the default rotation is gripper pointing along the negative z axis
            desiredRotation = np.array([
                [0, -1,  0],   
                [-1, 0,  0],   
                [0,  0, -1]
            ])
        if q_start is None:
            q_start = self.q_current
        desiredTransforms[0] = np.concatenate([desiredTranslation, desiredRotation.flatten()])
        self.ik_motion_solver.set_desired_endeffector_transforms(desiredTransforms)
        print("DEBUG", q_start)
        qs, if_success = self.ik_motion_solver.solve(q_start)
        
        if not if_success:
            raise Exception("IK failed")
        
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
        with self.endeffector_inertial_lock:
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
        if self.pick_up_done:
            return
            
        if self.current_task_index == 0:
            if self.trajectory_remaining_time == 0:
                pick_pos = self.pick_pos[self.dumbbell_order[self.dumbbell_id]]
                self.approach_q1 = self.IK(pick_pos + self.offset_x_neg, self.desiredRotation_x_pos, self.approach_q1)
                print(self.approach_q1)
                self.goto_near(self.approach_q1)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                self.trajectory_remaining_time = self.duration
                print("Approach above the object")
            else:
                self.trajectory_remaining_time -= self.SPIN_DURATION
                print("Waiting for pick up - approach motion to finish, remaining time:", self.trajectory_remaining_time)
                if self.trajectory_remaining_time <= 0:
                    self.current_task_index += 1
                    self.trajectory_remaining_time = 0.0
        elif self.current_task_index == 1:
            if self.trajectory_remaining_time == 0:
                pick_pos = self.pick_pos[self.dumbbell_order[self.dumbbell_id]]
                self.approach_q2 = self.IK(pick_pos, self.desiredRotation_x_pos, self.approach_q1)
                self.approach_q3 = self.IK(pick_pos + self.offset_z_pos, self.desiredRotation_x_pos, self.approach_q2)
                print(self.approach_q2)
                print(self.approach_q3)
                
                current_time = time.time()
                
                start_time_1 = current_time + 0.05
                
                self.goto_near(self.approach_q2, start_time_1, 2.0)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.01)
                print("Approach the object")
                
                start_time_2 = start_time_1 + 2.0
                self.open_or_close_gripper(False, start_time_2, self.approach_q2)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.01)
                print("Close the gripper")
                
                start_time_3 = start_time_2 + 2.5
                self.goto_near(self.approach_q3, start_time_3, 2.0, self.approach_q2)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.01)
                print("Pick up the object")
                
                # total time for the above three trajectories is 2.0 + 2.5 + 2.0 = 6.5
                self.trajectory_remaining_time = 6.5
            else:
                self.trajectory_remaining_time -= self.SPIN_DURATION
                print("Waiting for pick up - pick up motion to finish, remaining time:", self.trajectory_remaining_time)
                if self.trajectory_remaining_time <= 0:
                    self.current_task_index = 0
                    self.pick_up_done = True
                    self.identification_done = False
                    self.place_down_done = False
                    self.trajectory_remaining_time = 0.0
                
    def place_down_loop(self):
        if self.place_down_done:
            return
        
        if self.current_task_index <= 0:
            print("Wait until previous task is done")
            self.current_task_index += 1
        elif self.current_task_index == 1:
            if self.trajectory_remaining_time == 0:
                place_pos = self.place_pos[self.dumbbell_order[self.dumbbell_id]]
                self.place_q1 = self.IK(place_pos + self.offset_z_pos, self.desiredRotation_y_neg)
                print(self.place_q1)
                self.goto_near(self.place_q1)
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
        elif self.current_task_index == 2:
            if self.trajectory_remaining_time == 0:
                place_pos = self.place_pos[self.dumbbell_order[self.dumbbell_id]]
                self.place_q2 = self.IK(place_pos, self.desiredRotation_y_neg)
                self.place_q3 = self.IK(place_pos + self.offset_y_pos, self.desiredRotation_y_neg)
                print(self.place_q2)
                print(self.place_q3)
                
                current_time = time.time()
                
                start_time_1 = current_time + 0.05
                
                self.goto_near(self.place_q2, start_time_1)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.01)
                print("Approach the place position")
                
                start_time_2 = start_time_1 + self.duration
                
                self.open_or_close_gripper(True, start_time_2, self.place_q2)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.01)
                print("Release the gripper")
                
                start_time_3 = start_time_2 + 2.5
                
                self.goto_near(self.place_q3, start_time_3, 3.0, self.place_q2)
                self.traj_pub.publish(self.traj_msg)
                self.traj_id += 1
                time.sleep(0.01)
                print("Finish placing down")
                
                self.trajectory_remaining_time = self.duration + 2.5 + 3.0
            else:
                self.trajectory_remaining_time -= self.SPIN_DURATION
                print("Waiting for place down - place down motion to finish, remaining time:", self.trajectory_remaining_time)
                if self.trajectory_remaining_time <= 0:
                    self.current_task_index = 0
                    self.pick_up_done = False
                    self.identification_done = False
                    self.place_down_done = True
                    self.trajectory_remaining_time = 0.0
                    self.dumbbell_id += 1
                        
    def reset_sysid(self):
        self.exciting_id = 0
        self.exciting_q_targets = queue.Queue()
        q_targets_initial = pkl.load(open("q_targets_initial.pkl", "rb"))
        for q_target in q_targets_initial:
            self.exciting_q_targets.put(q_target)
        self.exciting_start_time = []
        self.sysid_solver.reset()
        self.identification_id = 0
        self.theta_lb = np.array([1.2, -0.1, -0.1, -0.8, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15])
        self.theta_ub = np.array([5.0, 0.1, 0.1, -0.1, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15])
        self.theta_solution = 0.5 * (self.theta_lb + self.theta_ub)
        self.theta_uncertainty = 0.5 * (self.theta_ub - self.theta_lb)
                    
    def plan_exciting_trajectory(self):
        print("Computing a exciting trajectory")
        
        counting = time.time()
        
        k_center = np.zeros(7)
        k_range = np.zeros(7)
        k_range[:4] = np.pi/48
        k_range[4:] = np.pi/20
        
        with self.exciting_initial_conditions_lock:
            exciting_q0 = copy.deepcopy(self.exciting_q0) # make a copy for later use
            self.planner.set_trajectory_parameters(
                exciting_q0, \
                self.exciting_qd0, \
                self.exciting_qdd0, \
                k_center, \
                k_range, \
                self.exciting_duration)
            
            print(exciting_q0, self.exciting_qd0, self.exciting_qdd0)
        
        if self.identification_id > 0:
            with self.endeffector_inertial_lock:
                print("Using the updated end effector inertial parameters")
                print(self.theta_solution, self.theta_lb, self.theta_ub)
                self.planner.set_endeffector_inertial_parameters(
                    self.theta_solution, \
                    self.theta_lb, \
                    self.theta_ub)
        
        k, ifFeasible = self.planner.optimize()
        
        if ifFeasible:
            exciting_q_target = exciting_q0 + k_center + k * k_range
        else:
            traj, sphere_xs, sphere_ys, sphere_zs, sphere_radii, torque_centers, torque_radii = self.planner.analyze_solution()
            np.savetxt("traj.txt", traj)
            np.savetxt("torque_radii.txt", torque_radii)
            raise Exception("Trajectory not feasible!")
        
        self.exciting_q_targets.put(exciting_q_target)
        
        computation_time = time.time() - counting
        print("Planning took: ", computation_time, " seconds")
        if computation_time > self.SPIN_DURATION:
            raise Exception("Planning took too long")
            
    def endeffector_identification(self, exciting_id, traj_id):
        print("Performing system identification")
        
        counting = time.time()
            
        # ignore the first exciting trajectory
        if exciting_id > 0:
            self.sysid_solver.add_trajectory_file('log/torque_output_' + str(traj_id) + '.txt')
        
        # perform system identification if there are enough trajectories
        if exciting_id >= 4 and exciting_id % 3 == 1:
            new_theta_solution, new_theta_uncertainty = self.sysid_solver.optimize()
            
            # update the end effector inertial parameters lower bound, upper bound
            # change the solution if it is out of the bounds
            with self.endeffector_inertial_lock:
                self.theta_solution = new_theta_solution
                self.theta_uncertainty = new_theta_uncertainty
            
                for i in range(10):
                    new_theta_lb = self.theta_solution[i] - self.theta_uncertainty[i]
                    new_theta_ub = self.theta_solution[i] + self.theta_uncertainty[i]
                    if new_theta_lb > self.theta_ub[i]:
                        raise Exception("Infeasible solution")
                    if new_theta_ub < self.theta_lb[i]:
                        raise Exception("Infeasible solution")
                    
                    if 1 <= i and i <= 2:
                        new_theta_lb = min(-0.01, new_theta_lb)
                        new_theta_ub = max(0.01, new_theta_ub)
                    
                    if new_theta_lb > self.theta_lb[i]:
                        self.theta_lb[i] = new_theta_lb
                    if new_theta_ub < self.theta_ub[i]:
                        self.theta_ub[i] = new_theta_ub
                
                print("theta solution:\n", self.theta_solution)
                print("theta uncertainty:\n", self.theta_uncertainty)
                print("theta lb:\n", self.theta_lb)
                print("theta ub:\n", self.theta_ub)
                
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
            
                self.identification_id += 1
        
        computation_time = time.time() - counting
        print("Identification took: ", computation_time, " seconds")
        if computation_time > self.SPIN_DURATION:
            raise Exception("Identification took too long")

    def plan(self):
        counting = time.time()
        
        endeffector_identification_thread = None
        if self.current_task_index > 0:
            # perform system identification on the previous trajectory that has been executed
            # self.theta_solution, self.theta_lb, self.theta_ub are updated here
            endeffector_identification_thread = threading.Thread(
                target=self.endeffector_identification, args=(self.exciting_id, self.traj_id - 1))
            endeffector_identification_thread.start()
        
        # something went wrong with planning, so no new target received
        # terminate the sysid process now
        if self.exciting_q_targets.empty():
            self.current_task_index = 0
            self.identification_done = True
            
            # reset the identification process
            if endeffector_identification_thread is not None:
                endeffector_identification_thread.join()
            print("No exciting trajectories anymore! Resetting system identification")
            self.reset_sysid()
            return
        
        # send the current exciting trajectory to the robot
        current_exciting_q_target = self.exciting_q_targets.get()
        
        if self.current_task_index == 0: # the robot is stopping right now, send the first exciting trajectory
            current_time = time.time()
            self.exciting_start_time = current_time + 0.05
            self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                self.exciting_start_time, 
                self.exciting_duration, 
                0.5 * self.exciting_duration, 
                self.q_current,
                self.qd_current,
                np.zeros(7),
                current_exciting_q_target,
                self.gripper_state)
        else: # the robot is still moving, so connect with the previous trajectory
            self.traj_msg = trajectory_helper.formulate_armour_trajectory_message_continuous_with_previous(
                self.exciting_start_time,
                self.exciting_duration,
                0.5 * self.exciting_duration,
                current_exciting_q_target,
                self.traj_msg)
            
        self.traj_pub.publish(self.traj_msg)
        time.sleep(0.01)
        self.exciting_start_time += 0.5 * self.exciting_duration
        
        # update initial conditions for the next trajectory
        with self.exciting_initial_conditions_lock:
            trajectory_compute.setup(
                self.traj_msg.start_time, 
                self.exciting_duration, 
                self.exciting_duration, 
                7, 
                TrajectoryMacros.ARMOUR_TRAJ, 
                self.traj_msg.traj_data)
            self.exciting_q0, self.exciting_qd0, self.exciting_qdd0 = trajectory_compute.compute(self.exciting_start_time)
        
        with self.trajectory_data_lock:
            self.traj_id += 1
            self.exciting_id += 1
        
        plan_exciting_trajectory_thread = None
        if self.identification_id >= 3:
            self.current_task_index = 0
            self.identification_done = True
            
            # reset the identification process
            if endeffector_identification_thread is not None:
                endeffector_identification_thread.join()
            print("Enough identification finished! Resetting system identification")
            self.reset_sysid()
            return
        else:
            if self.exciting_q_targets.empty():
                plan_exciting_trajectory_thread = threading.Thread(target=self.plan_exciting_trajectory, args=())
                plan_exciting_trajectory_thread.start()
             
        if endeffector_identification_thread is not None:
            endeffector_identification_thread.join()
            
            # change the end effector inertial parameters once the system identification is done
            # self.change_endeffector_inertial()
        if plan_exciting_trajectory_thread is not None:
            plan_exciting_trajectory_thread.join()
             
        self.current_task_index += 1
        
        if time.time() - counting > self.SPIN_DURATION:
            raise Exception("Planning and identification together took too long")
                    
    def _timer_callback(self) -> None:
        if not self.pick_up_done:
            self.pick_up_loop()
            print(self.pick_up_done, ' pick up', self.traj_id)
        # elif not self.identification_done:    
        #     self.plan()
        #     print(self.identification_done, ' exciting trajectory', self.traj_id)
        else:
            self.place_down_loop()
            print(self.place_down_done, ' place down', self.traj_id)
            
        if self.dumbbell_id >= len(self.dumbbell_order):
            raise Exception("All tasks finished")

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()