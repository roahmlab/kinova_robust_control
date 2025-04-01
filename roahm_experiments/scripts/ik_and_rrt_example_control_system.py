#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from roahm_msgs.msg import KortexMeasurements, TrajectoryMsg
import sys
import logging
import time

from roahm_trajectories import TrajectoryMacros

sys.path.append("/workspaces/kinova_control_docker/src/RAPTOR/build/lib")
import KinovaIKMotion_nanobind
import KinovaHLP_nanobind

# obstacle information (xyz, rpy, size)
obstacles = np.array([[0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 0.01], # ground
                      [0.53, 0.49, 0.56, 0.0, 0.0, 0.0, 2.0, 0.08, 1.12], # back wall
                      [-0.39, -0.82, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # camera bar near the control
                      [-0.39, 0.42, 0.56, 0.0, 0.0, 0.0, 0.08, 0.08, 1.12], # second camera bar
                      [0.7, 0.0, 1.12, 0.0, 0.0, 0.0, 2.0, 2.0, 0.05] # ceiling
                     ])

urdf_filename = "./models/urdf/gen3_2f85.urdf"

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
        
        self.q_current = np.zeros(7)
        self.qd_current = np.zeros(7)
        self.q_target = np.zeros(7)
        self.q_prev_target = np.zeros(7)
        
        self.ik_motion_solver = KinovaIKMotion_nanobind.KinovaIKMotionPybindWrapper(urdf_filename, False)
        self.rrtplanner = KinovaHLP_nanobind.WaypointPlanningPybindWrapper(urdf_filename)

        # ipopt settings
        ipopt_tol = 1e-6
        ipopt_constr_viol_tol = 1e-8
        ipopt_obj_scaling_factor = 1.0
        ipopt_max_wall_time = 1.5
        ipopt_print_level = 0
        ipopt_mu_strategy = "monotone"
        ipopt_linear_solver = "ma86"
        ipopt_gradient_check = False

        # set up ik solver
        self.ik_motion_solver.set_ipopt_parameters(
            ipopt_tol, \
            ipopt_constr_viol_tol, \
            ipopt_obj_scaling_factor, \
            ipopt_max_wall_time, \
            ipopt_print_level, \
            ipopt_mu_strategy, \
            ipopt_linear_solver,
            ipopt_gradient_check)
        
        self.q_init = np.array([2.1330967, 0.92962713, 2.09719171, -1.57763623, 0.86096777, -1.15192888, 0.89113271])
        
        # set up rrt planner
        self.rrt_collision_buffer = 0.0 # m
        self.include_gripper_or_not = True
        self.rrtplanner.set_obstacles(obstacles, self.rrt_collision_buffer)
        self.current_waypoint_index = 0
        
        self.duration = 3.0
        self.time_start = []
        self.traj_id = 0
        
        # predefined pick up positions in the arm frame
        approach_pos = np.array([0.0, -0.5, 0.25])
        place_pos = np.array([0.5, 0.0, 0.25])

        # solve IK for approach and retract configurations
        self.approach_q  = self.IK(approach_pos)
        self.place_q = self.IK(place_pos)

        print(self.approach_q)
        print(self.place_q)
        
        logging.info("RRT planning")
        print("RRT planning")
        self.RRTPlan(self.approach_q, self.place_q)
        
        self.create_timer(self.duration, self._timer_callback)
        
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
            [1, 0,  0],   
            [0, -1,  0],   
            [0, 0, -1]   
        ])
        desiredTransforms[0] = np.concatenate([desiredTranslation, desiredRotation.flatten()])
        
        self.ik_motion_solver.set_desired_endeffector_transforms(desiredTransforms)
        
        qs, if_success = self.ik_motion_solver.solve(self.q_init)
        
        return qs.flatten()
    
    def RRTPlan(self, start, goal):
        self.rrtplanner.set_start_goal(start, goal)
        try:
            rrtplanner_time = 2.0 # sec
            self.rrtpath = self.rrtplanner.plan(rrtplanner_time, self.include_gripper_or_not)
            print(len(self.rrtpath))
        except Exception as e:
            print(e)
            self.rrtpath = []
            
    def execute_motion(self):
        current_time = time.time()
        
        if self.traj_id == 0:
            logging.info("Moving to approach position")
            print("Moving to approach position")
            
            self.q_prev_target = self.q_current
            self.q_target = self.approach_q
            self.time_start = current_time + 0.025
        else:
            logging.info("Executing RRT path")
            print("Executing RRT path")
            
            self.current_waypoint_index += 10
            self.q_prev_target = self.q_target
            if self.current_waypoint_index >= len(self.rrtpath):
                self.q_target = np.array(self.rrtpath[-1, :])
                self.current_waypoint_index = len(self.rrtpath)
            else:
                self.q_target = np.array(self.rrtpath[self.current_waypoint_index, :])
            self.time_start = self.time_start + self.duration

        logging.info("Publishing a trajectory")
        print("Publishing a trajectory")

        self.traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
        for i in range(7):
            self.traj_msg.traj_data[4 * i + 0] = self.q_prev_target[i]
            self.traj_msg.traj_data[4 * i + 1] = 0.0
            self.traj_msg.traj_data[4 * i + 2] = 0.0
            self.traj_msg.traj_data[4 * i + 3] = self.q_target[i]
        
        self.traj_msg.start_time = self.time_start
        self.traj_msg.trajectory_duration = self.duration
        self.traj_msg.duration = self.duration
        
        self.traj_msg.dof = 7
        self.traj_msg.trajectory_type = TrajectoryMacros.ARMOUR_TRAJ

        self.traj_msg.is_gripper_open = True
        self.traj_msg.reset = False

        # publish
        self.traj_pub.publish(self.traj_msg)
        self.traj_id += 1

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
        
        if self.current_waypoint_index == len(self.rrtpath):
            raise Exception("RRT path finished.")
            
    def _timer_callback(self) -> None:
        self.execute_motion()

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()