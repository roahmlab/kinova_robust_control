#!/usr/bin/env python3

import numpy as np
from enum import Enum, auto
from typing import Optional
from collections import deque
import pinocchio as pin
import time

import rclpy
from rclpy.node import Node
from customized_msgs.msg import KortexMeasurements, TrajectoryMsg

import trajectory_helper

import sys, select, termios, tty
sys.path.append('src/RAPTOR/build/lib')
import KinovaIKMotion_nanobind

# obstacle information (xyz, rpy, size)
obstacles = np.array([[0.51, 0.0, 0.15, 0.0, 0.0, 0.0, 0.30, 0.30, 0.30], # one obstacle in front of the robot
                      [0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 0.01], # ground
                     ])

class TestNode(Node):
    def __init__(self):
        super().__init__("trajectory_test_node")
        # trajectory publisher
        self.traj_pub = self.create_publisher(
            TrajectoryMsg, 
            "/trajectory", # should be consistent with "ros_traj_topic" in config.yaml
            10)
        self.traj_msg: TrajectoryMsg = TrajectoryMsg()

        # joint measurement
        self.joint_info_sub = self.create_subscription(
            KortexMeasurements, 
            "/joint_info", 
            self._joint_info_callback, 
            10)
        
        # initialize inverse kinematics solver
        urdf_path = 'models/urdf/gen3_2f85_fixed.urdf'
        self.ik_solver = KinovaIKMotion_nanobind.KinovaIKMotionPybindWrapper(urdf_path, False)
        self.ik_solver.set_obstacles(obstacles, 0.02)
        self.ik_solver.set_ipopt_parameters(
            1e-10,          # tol
            1e-6,           # constr_viol_tol
            10,             # obj_scaling_factor
            0.1,            # max_wall_time
            0,              # print_level
            "monotone",     # mu_strategy
            "ma86",         # linear_solver
            False           # gradient_check
        )
        
        # initialize pinocchio forward kinematics solver
        self.model = pin.buildModelFromUrdf(urdf_path)
        
        # add contact frame (middle of the gripper) to the model
        endT1 = pin.SE3(
            np.array([[1, 0, 0], 
                      [0, -1, 0], 
                      [0, 0, -1]]), 
            np.array([0, 0, -0.061525])) # end effector -> gripper base
        
        endT2 = pin.SE3(
            np.array([[0, -1, 0], 
                      [1, 0, 0], 
                      [0, 0, 1]]), 
            np.array([0, 0, 0.12])) # gripper base -> contact joint
        
        endT = endT1 * endT2
        
        last_joint_id = self.model.getJointId("joint_7")
        self.model.addFrame(
            pin.Frame(
                "gripper_frame", 
                last_joint_id, 
                0, 
                endT, 
                pin.FrameType.OP_FRAME
            )
        )
        
        self.end_effector_id = self.model.getFrameId("gripper_frame")
        self.data = self.model.createData()
        
        self.if_initialized = False # flag to check if the initial end effector frame is initialized
        
        self.create_timer(
            0.5, # send a 0.3 second trajectory every 0.5 seconds if key is pressed
            self._timer_callback)

    def _joint_info_callback(self, msg) -> None:
        """
        callback function for joint_info
        """
        self.q_current = np.array(msg.pos)
        self.qd_current = np.array(msg.vel)

    def publish_test_trajectory(self):
        """
        Read keyboard input and publish a corresponding teleoperation trajectory
        """
        
        # check if the end effector frame is initialized
        if not self.if_initialized:
            pin.forwardKinematics(self.model, self.data, self.q_current)
            pin.updateFramePlacements(self.model, self.data)
            initial_frame = self.data.oMf[self.end_effector_id]
            self.p = initial_frame.translation
            self.R = initial_frame.rotation
            self.if_initialized = True
            
        # read keyboard input
        def get_key(timeout=0.05):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                rlist, _, _ = select.select([fd], [], [], timeout)
                if rlist:
                    return sys.stdin.read(1)
                else:
                    return None
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        key = get_key(0.05)
        if key is None:
            # print("No key pressed, skipping publishing")
            return
        else:
            print(f"Key pressed: {key}")
        
        delta_p = np.zeros(3)
        delta_R = pin.SE3.Identity().rotation 
        
        # translation (2 cm)
        if key == 'w':
            delta_p[0] = 0.02
        elif key == 's':
            delta_p[0] = -0.02
        elif key == 'a':
            delta_p[1] = 0.02
        elif key == 'd':
            delta_p[1] = -0.02
        elif key == 'q':
            delta_p[2] = 0.02
        elif key == 'e':
            delta_p[2] = -0.02
            
        # rotation (2.5 degrees)
        elif key == 'j': # yaw left
            delta_R = pin.rpy.rpyToMatrix(0, 0, 2.5 * np.pi  / 180.0)
        elif key == 'l': # yaw right
            delta_R = pin.rpy.rpyToMatrix(0, 0, -2.5 * np.pi  / 180.0)
        elif key == 'i': # pitch up
            delta_R = pin.rpy.rpyToMatrix(0, 2.5 * np.pi  / 180.0, 0)
        elif key == 'k': # pitch down
            delta_R = pin.rpy.rpyToMatrix(0, -2.5 * np.pi  / 180.0, 0)
        elif key == 'u': # roll counter-clockwise
            delta_R = pin.rpy.rpyToMatrix(2.5 * np.pi  / 180.0, 0, 0)
        elif key == 'o': # roll clockwise
            delta_R = pin.rpy.rpyToMatrix(-2.5 * np.pi  / 180.0, 0, 0)
            
        self.p += delta_p
        self.R = self.R @ delta_R

        # solve inverse kinematics
        self.ik_solver.set_desired_endeffector_transforms(
            np.concatenate((self.p, self.R.flatten()))[np.newaxis, :]
        )
        
        print(self.q_current)
        self.q_goal, has_optimized = self.ik_solver.solve(self.q_current)
        
        
        print(self.q_goal.flatten())
        
        if not has_optimized:
            raise RuntimeError("IK solver failed to find a solution")

        self.traj_msg = trajectory_helper.formulate_armour_trajectory_message(
            time.time() + 0.02,     # start time
            0.3,                    # trajectory duration
            0.3,                    # replay duration
            self.q_current,         # current joint position
            self.qd_current,        # current joint velocity
            np.zeros(7),            # current joint acceleration
            self.q_goal.flatten()   # goal joint position
        )

        # publish
        self.traj_pub.publish(self.traj_msg)

    def _timer_callback(self) -> None:
        self.publish_test_trajectory()

if __name__ == "__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()