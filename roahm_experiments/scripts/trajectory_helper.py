#!/usr/bin/env python3

import numpy as np
import time

import rclpy
from rclpy.node import Node
from roahm_msgs.msg import KortexMeasurements, TrajectoryMsg

import roahm_trajectories_py
from roahm_trajectories import TrajectoryMacros

def refine_angdiff_for_kinova(angdiff: np.ndarray) -> np.ndarray:
    for i in np.array([0, 2, 4, 6]):
        if angdiff[i] > np.pi:
            angdiff[i] -= 2 * np.pi
        elif angdiff[i] < -np.pi:
            angdiff[i] += 2 * np.pi
    return np.linalg.norm(angdiff)

def refine_target(
    current: float, 
    target: float) -> float:
    angdiff = target - current
    if angdiff > np.pi:
        target -= 2 * np.pi
    elif angdiff < -np.pi:
        target += 2 * np.pi
    return target

def refine_targets_for_kinova(
    current: np.ndarray, 
    target: np.ndarray) -> np.ndarray:
    for i in np.array([0, 2, 4, 6]):
        target[i] = refine_target(current[i], target[i])
    return target

def formulate_armtd_trajectory_message(
    start_time: float, 
    trajectory_duration: float,
    duration: float, 
    q0: np.ndarray,
    q_d0: np.ndarray,
    k: np.ndarray,
    t_brake: float,
    is_gripper_open: bool = True) -> TrajectoryMsg:
    if len(q0) != 7 or \
       len(q_d0) != 7 or \
       len(k) != 7:
        raise ValueError("Invalid input shape")

    traj_msg = TrajectoryMsg()

    traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
    for i in range(7):
        traj_msg.traj_data[3 * i + 0] = q0[i]
        traj_msg.traj_data[3 * i + 1] = q_d0[i]
        traj_msg.traj_data[3 * i + 2] = k[i]
    traj_msg.traj_data[3 * 7] = t_brake

    traj_msg.start_time = start_time
    traj_msg.trajectory_duration = trajectory_duration
    traj_msg.duration = duration

    traj_msg.dof = 7
    traj_msg.trajectory_type = TrajectoryMacros.ARMTD_TRAJ

    traj_msg.is_gripper_open = is_gripper_open
    traj_msg.reset = False

    return traj_msg

def formulate_armour_trajectory_message(
    start_time: float, 
    trajectory_duration: float,
    duration: float, 
    q0: np.ndarray,
    q_d0: np.ndarray,
    q_dd0: np.ndarray,
    qT: np.ndarray,
    is_gripper_open: bool = True) -> TrajectoryMsg:
    if len(q0) != 7 or \
       len(q_d0) != 7 or \
       len(q_dd0) != 7 or \
       len(qT) != 7:
        raise ValueError("Invalid input shape")

    traj_msg = TrajectoryMsg()
    
    qT = refine_targets_for_kinova(q0, qT)

    traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
    for i in range(7):
        traj_msg.traj_data[4 * i + 0] = q0[i]
        traj_msg.traj_data[4 * i + 1] = q_d0[i]
        traj_msg.traj_data[4 * i + 2] = q_dd0[i]
        traj_msg.traj_data[4 * i + 3] = qT[i]

    traj_msg.start_time = start_time
    traj_msg.trajectory_duration = trajectory_duration
    traj_msg.duration = duration

    traj_msg.dof = 7
    traj_msg.trajectory_type = TrajectoryMacros.ARMOUR_TRAJ

    traj_msg.is_gripper_open = is_gripper_open
    traj_msg.reset = False

    return traj_msg

def formulate_Bezier_trajectory_message(
    start_time: float, 
    trajectory_duration: float,
    duration: float, 
    q0: np.ndarray,
    q_d0: np.ndarray,
    q_dd0: np.ndarray,
    qT: np.ndarray,
    q_dT: np.ndarray,
    q_ddT: np.ndarray,
    is_gripper_open: bool = True) -> TrajectoryMsg:
    if len(q0) != 7 or \
       len(q_d0) != 7 or \
       len(q_dd0) != 7 or \
       len(qT) != 7 or \
       len(q_dT) != 7 or \
       len(q_ddT) != 7:
        raise ValueError("Invalid input shape")

    traj_msg = TrajectoryMsg()
    
    qT = refine_targets_for_kinova(q0, qT)

    traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
    for i in range(7):
        traj_msg.traj_data[6 * i + 0] = q0[i]
        traj_msg.traj_data[6 * i + 1] = q_d0[i]
        traj_msg.traj_data[6 * i + 2] = q_dd0[i]
        traj_msg.traj_data[6 * i + 3] = qT[i]
        traj_msg.traj_data[6 * i + 4] = q_dT[i]
        traj_msg.traj_data[6 * i + 5] = q_ddT[i]

    traj_msg.start_time = start_time
    traj_msg.trajectory_duration = trajectory_duration
    traj_msg.duration = duration

    traj_msg.dof = 7
    traj_msg.trajectory_type = TrajectoryMacros.FIFTH_ORDER_BEZIER_TRAJ

    traj_msg.is_gripper_open = is_gripper_open
    traj_msg.reset = False

    return traj_msg

def formulate_Fourier_trajectory_message(
    start_time: float, 
    trajectory_duration: float,
    duration: float, 
    traj_data: np.ndarray,
    is_gripper_open: bool = True) -> TrajectoryMsg:
    if len(traj_data) > TrajectoryMacros.TRAJECTORY_DATA_SIZE:
        raise ValueError("Invalid input shape")

    traj_msg = TrajectoryMsg()

    traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
    traj_msg.traj_data[:len(traj_data)] = traj_data

    traj_msg.start_time = start_time
    traj_msg.trajectory_duration = trajectory_duration
    traj_msg.duration = duration

    traj_msg.dof = 7
    traj_msg.trajectory_type = TrajectoryMacros.FOURIER_TRAJ

    traj_msg.is_gripper_open = is_gripper_open
    traj_msg.reset = False

    return traj_msg 

def formulate_armtd_trajectory_message_continuous_with_previous(
    start_time: float, 
    trajectory_duration: float,
    duration: float, 
    k: np.ndarray,
    t_brake: float,
    traj_msg_prev: TrajectoryMsg) -> TrajectoryMsg:
    traj_msg = TrajectoryMsg()

    traj_prev = roahm_trajectories_py.TrajectoryPybindWrapper()
    traj_prev.setup(traj_msg_prev.start_time, 
                    traj_msg_prev.trajectory_duration, 
                    traj_msg_prev.duration, 
                    traj_msg_prev.dof, 
                    traj_msg_prev.trajectory_type, 
                    traj_msg_prev.traj_data)
    
    q0, q_d0, _ = traj_prev.compute(start_time)

    traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
    for i in range(7):
        traj_msg.traj_data[3 * i + 0] = q0[i]
        traj_msg.traj_data[3 * i + 1] = q_d0[i]
        traj_msg.traj_data[3 * i + 2] = k[i]
    traj_msg.traj_data[3 * 7] = t_brake

    traj_msg.start_time = start_time
    traj_msg.trajectory_duration = trajectory_duration
    traj_msg.duration = duration

    traj_msg.dof = 7
    traj_msg.trajectory_type = TrajectoryMacros.ARMTD_TRAJ

    traj_msg.is_gripper_open = traj_msg_prev.is_gripper_open
    traj_msg.reset = False

    return traj_msg

def formulate_armour_trajectory_message_continuous_with_previous(
    start_time: float, 
    trajectory_duration: float,
    duration: float, 
    qT: np.ndarray,
    traj_msg_prev: TrajectoryMsg) -> TrajectoryMsg:
    traj_msg = TrajectoryMsg()

    traj_prev = roahm_trajectories_py.TrajectoryPybindWrapper()
    traj_prev.setup(traj_msg_prev.start_time, 
                    traj_msg_prev.trajectory_duration, 
                    traj_msg_prev.duration, 
                    traj_msg_prev.dof, 
                    traj_msg_prev.trajectory_type, 
                    traj_msg_prev.traj_data)
    
    q0, q_d0, q_dd0 = traj_prev.compute(start_time)
    
    qT = refine_targets_for_kinova(q0, qT)

    traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
    for i in range(7):
        traj_msg.traj_data[4 * i + 0] = q0[i]
        traj_msg.traj_data[4 * i + 1] = q_d0[i]
        traj_msg.traj_data[4 * i + 2] = q_dd0[i]
        traj_msg.traj_data[4 * i + 3] = qT[i]

    traj_msg.start_time = start_time
    traj_msg.trajectory_duration = trajectory_duration
    traj_msg.duration = duration

    traj_msg.dof = 7
    traj_msg.trajectory_type = TrajectoryMacros.ARMOUR_TRAJ

    traj_msg.is_gripper_open = traj_msg_prev.is_gripper_open
    traj_msg.reset = False

    return traj_msg

def formulate_Bezier_trajectory_message_continuous_with_previous(
    start_time: float,
    trajectory_duration: float,
    duration: float,
    qT: np.ndarray,
    q_dT: np.ndarray,
    q_ddT: np.ndarray,
    traj_msg_prev: TrajectoryMsg) -> TrajectoryMsg:
    traj_msg = TrajectoryMsg()

    traj_prev = roahm_trajectories_py.TrajectoryPybindWrapper()
    traj_prev.setup(traj_msg_prev.start_time, 
                    traj_msg_prev.trajectory_duration, 
                    traj_msg_prev.duration, 
                    traj_msg_prev.dof, 
                    traj_msg_prev.trajectory_type, 
                    traj_msg_prev.traj_data)
    
    q0, q_d0, q_dd0 = traj_prev.compute(start_time)
    
    qT = refine_targets_for_kinova(q0, qT)

    traj_msg.traj_data = np.zeros(TrajectoryMacros.TRAJECTORY_DATA_SIZE, dtype=np.float64)
    for i in range(7):
        traj_msg.traj_data[6 * i + 0] = q0[i]
        traj_msg.traj_data[6 * i + 1] = q_d0[i]
        traj_msg.traj_data[6 * i + 2] = q_dd0[i]
        traj_msg.traj_data[6 * i + 3] = qT[i]
        traj_msg.traj_data[6 * i + 4] = q_dT[i]
        traj_msg.traj_data[6 * i + 5] = q_ddT[i]

    traj_msg.start_time = start_time
    traj_msg.trajectory_duration = trajectory_duration
    traj_msg.duration = duration

    traj_msg.dof = 7
    traj_msg.trajectory_type = TrajectoryMacros.FIFTH_ORDER_BEZIER_TRAJ

    traj_msg.is_gripper_open = traj_msg_prev.is_gripper_open
    traj_msg.reset = False

    return traj_msg