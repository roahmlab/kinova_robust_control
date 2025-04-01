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
import armour_nanobind
import KinovaIKMotion_nanobind
import KinovaHLP_nanobind

# obstacle information (xyz, rpy, size)
obstacles = np.array([[0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 0.01], # ground
                      [0.53, 0.49, 0.56, 0.0, 0.0, 0.0, 2.0, 0.08, 1.12], # back wall
                      [-0.39, -0.82, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # camera bar near the control
                      [-0.39, 0.42, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # second camera bar
                      [0.7, 0.0, 1.12, 0.0, 0.0, 0.0, 2.0, 2.0, 0.05] # ceiling
                     ])

urdf_filename = "./models/urdf/gen3_2f85_fixed.urdf"
config_filename = "src/RAPTOR/Examples/Kinova/Armour/KinovaWithGripperInfo.yaml"

trajectory_compute = roahm_trajectories_py.TrajectoryPybindWrapper()

class TestNode(Node):
    def __init__(self):
        super().__init__("armour_trajectories_node")
        # trajectory publisher
        self.traj_pub = self.create_publisher(
            TrajectoryMsg, "/trajectory", 10)
        self.traj_msg: TrajectoryMsg = TrajectoryMsg()

        # joint measurement
        self.joint_info_sub = self.create_subscription(
            KortexMeasurements, "/joint_info", self._joint_info_callback, 10)
        
        self.q_current = np.zeros(7)
        self.qd_current = np.zeros(7)
        self.q_target = np.zeros(7)
        self.q_prev_target = np.zeros(7)
        
        # start and goal configurations
        self.q_start = np.array([0.0, 0.262, 3.142, -2.2692, 0.0, -0.6108, 1.571])
        self.q_end = np.array([1.46704563, 1.07964977, -2.99617242, -2.03025423, -0.03453388, 1.53026597, -0.12680579])
        
        # set up rrt planner
        self.rrtplanner = KinovaHLP_nanobind.WaypointPlanningPybindWrapper(urdf_filename)
        self.rrt_collision_buffer = 0.05 # m
        self.include_gripper_or_not = True
        self.rrtplanner.set_obstacles(obstacles, self.rrt_collision_buffer)
        
        self.one_step_length = 0.5 # distance in robot configuration space between two waypoints
        self.current_waypoint_index = 0
        
        # set up armour planner
        display_info = True
        self.planner = armour_nanobind.ArmourPybindWrapper(urdf_filename, config_filename, display_info)
        
        self.planner.set_obstacles(obstacles)
        
        ipopt_tol = 1e-5
        ipopt_constr_viol_tol = 1e-6
        ipopt_obj_scaling_factor = 10.0
        ipopt_max_wall_time = 0.8 # sec
        ipopt_print_level = 5 # change to 5 for more information
        ipopt_mu_strategy = "monotone" # adaptive (more aggressive), monotone (more conservative)
        ipopt_linear_solver = "ma86" # ma57, ma86
        ipopt_gradient_check = False
        self.planner.set_ipopt_parameters(
            ipopt_tol, \
            ipopt_constr_viol_tol, \
            ipopt_obj_scaling_factor, \
            ipopt_max_wall_time, \
            ipopt_print_level, \
            ipopt_mu_strategy, \
            ipopt_linear_solver,
            ipopt_gradient_check)
        
        self.duration = 3.0 # sec, duration of each armour trajectory
        
        # finite state machine
        self.start_time = None # start time of the current receding horizon trajectory
        self.action_id = 0
        
        print("RRT planning")
        self.RRTPlan(self.q_start, self.q_end)
        
        self.create_timer(0.5 * self.duration, self._timer_callback)
    
    def RRTPlan(self, start, goal):
        """
        RRT path planning
        """
        self.rrtplanner.set_start_goal(start, goal)
        try:
            # rrtplanner_time = 2.0 # sec, maximum time for RRT planner
            # self.rrtpath = self.rrtplanner.plan(rrtplanner_time, self.include_gripper_or_not)
            # print("Number of configurations in the RRT path: ", len(self.rrtpath))
            
            # print(self.rrtpath.shape)
            # np.savetxt("rrt_path.txt", self.rrtpath)
            
            self.rrtpath = np.loadtxt("rrt_path.txt")
            
            self.rrtpath_length = 0
            for i in range(len(self.rrtpath)-1):
                self.rrtpath_length += np.linalg.norm(self.rrtpath[i+1] - self.rrtpath[i])
            print("RRT path length: ", self.rrtpath_length)
            
            self.num_final_waypoints = int(self.rrtpath_length / self.one_step_length)
            self.waypoint_step = int(len(self.rrtpath) / self.num_final_waypoints)
            
            print("Number of final chosen waypoints: ", self.num_final_waypoints)
            print("Waypoint step: ", self.waypoint_step)
            
            self.q_rrt_target = self.rrtpath[self.waypoint_step]
                
        except Exception as e:
            print(e)
            self.rrtpath = []
            raise Exception("RRT path planning failed.")
     
    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
        self.q_current = np.array(msg.pos)
        self.qd_current = np.array(msg.vel)
            
    def execute_motion(self):
        """
        Plan for a provably-safe trajectory to get to the next waypoint using Armour
        """
        
        counting = time.time()
        
        if self.action_id == 0:
            # plan for the first Armour trajectory and do not move the robot
            self.q0 = self.q_current
            self.qd0 = self.qd_current
            self.qdd0 = np.zeros(7)
        else:
            if self.action_id == 1: # send the first Armour trajectory
                self.start_time = time.time() + 0.05
            else:
                self.start_time += 0.5 * self.duration
                
            self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
                self.start_time, 
                self.duration, 
                0.5 * self.duration, 
                self.q0, 
                self.qd0, 
                self.qdd0, 
                self.q_target)
            self.traj_pub.publish(self.traj_msg)
        
            # update initial conditions for the next trajectory
            trajectory_compute.setup(
                self.traj_msg.start_time, 
                self.duration, 
                self.duration, 
                7, 
                TrajectoryMacros.ARMOUR_TRAJ, 
                self.traj_msg.traj_data)
            self.q0, self.qd0, self.qdd0 = trajectory_compute.compute(self.start_time + 0.5 * self.duration)
            
        distance = trajectory_helper.refine_angdiff_for_kinova(self.q_current - self.q_end)
        print("Distance to the goal: ", distance)
        if distance < 0.05:
            raise Exception("RRT path finished.")
        
        distance = trajectory_helper.refine_angdiff_for_kinova(self.q_current - self.q_rrt_target)
        print("Distance to the next waypoint: ", distance)
        
        is_last_waypoint = False
        if distance < np.pi/6:
            self.current_waypoint_index += self.waypoint_step
            print(f'Waypoint index: {self.current_waypoint_index} / {len(self.rrtpath)}')
            if self.current_waypoint_index >= len(self.rrtpath):
                self.q_rrt_target = self.q_end
                is_last_waypoint = True
                print("Next waypoint will be the final goal: ", self.q_rrt_target)
            else:
                self.q_rrt_target = self.rrtpath[self.current_waypoint_index]
                print("Proceed to next waypoint: ", self.q_rrt_target)
        
        # set up the local planner
        # self.k_center = np.pi/6 * (self.q_rrt_target - self.q0) / np.linalg.norm(self.q_rrt_target - self.q0)
        self.k_center = np.zeros(7)
        self.k_range = np.pi/12 * np.ones(7)
        self.q_plan = 0.5 * self.duration
        
        if is_last_waypoint:
            self.q_plan = self.duration
            
        self.planner.set_trajectory_parameters(
            self.q0, \
            self.qd0, \
            self.qdd0, \
            self.k_center, \
            self.k_range, \
            self.duration, \
            self.q_rrt_target, \
            self.q_plan)
        
        # plan for the Armour trajectory
        k, ifFeasible = self.planner.optimize()
        print('k', k)
        
        if not ifFeasible:
            raise Exception("Armour trajectory optimization failed.")
        
        self.q_target = self.q0 + self.k_center + k * self.k_range
        print('q_target', self.q_target)
        
        if time.time() - counting > 0.5 * self.duration:
            raise Exception("Armour trajectory optimization took too long.")
            
    def _timer_callback(self) -> None:
        self.execute_motion()
        self.action_id += 1

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()