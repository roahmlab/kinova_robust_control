#!/usr/bin/env python3

import argparse
import numpy as np
import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from roahm_msgs.msg import KortexMeasurements, TrajectoryMsg

import roahm_trajectories_py
from roahm_trajectories import TrajectoryMacros

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores, get_types_from_msg

STRIDX_MSG_JOINT_INFO = """
uint32 frame_id
builtin_interfaces/Time stamp
float32[7] pos
float32[7] vel
float32[7] torque
"""

STRIDX_MSG_TRAJECTORY = """
float64[100] traj_data
float64 start_time
float64 trajectory_duration
float64 duration
int32 dof
int32 trajectory_type
bool is_gripper_open
bool reset
"""

# bag_name = "rosbag2_2024_07_17-23_34_58"

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process the path to a ROS bag.')
    parser.add_argument('bag_name', type=str, help='The name of the ROS bag file to process.')
    args = parser.parse_args()

    bag_path = "/home/baiyuew/ROAHM/planning_wksp/" + args.bag_name + "/"

    # joint info
    t = np.array([])
    q = np.array([])
    q_d = np.array([])
    tau = np.array([])

    # desired trajectory
    td = np.array([])
    qd = np.array([])
    qd_d = np.array([])
    qd_dd = np.array([])

    def timestamp_to_sec(timestamp):
        return timestamp.sec + timestamp.nanosec * 1e-9

    traj_py = roahm_trajectories_py.TrajectoryPybindWrapper()

    # create reader instance and open for reading
    with Reader(bag_path) as reader:
        # Instantiate type store
        typestore = get_typestore(Stores.ROS2_FOXY)
        typestore.register(get_types_from_msg(STRIDX_MSG_JOINT_INFO, 'roahm_msgs/msg/KortexMeasurements'))
        typestore.register(get_types_from_msg(STRIDX_MSG_TRAJECTORY, 'roahm_msgs/msg/TrajectoryMsg'))

        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)

        # iterate over messages to recover joint trajectories
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/joint_info':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                if len(t) == 0:
                    t = np.array([timestamp_to_sec(msg.stamp)])
                    q = np.reshape(msg.pos, (1,7))
                    q_d = np.reshape(msg.vel, (1,7))
                    tau = np.reshape(msg.torque, (1,7))
                else:
                    t = np.append(t, timestamp_to_sec(msg.stamp))
                    q = np.concatenate((q, np.reshape(msg.pos, (1,7))), axis=0)
                    q_d = np.concatenate((q_d, np.reshape(msg.vel, (1,7))), axis=0)
                    tau = np.concatenate((tau, np.reshape(msg.torque, (1,7))), axis=0)

        # iterate over messages to collect all trajectory messages
        msgs = ()
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/trajectory':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                msgs = msgs + (msg,)
        
        # recover desired trajectories
        for i, msg in enumerate(msgs):
            traj_py.setup(msg.start_time, 
                          msg.trajectory_duration, 
                          msg.duration, 
                          msg.dof, 
                          msg.trajectory_type, 
                          msg.traj_data)
            
            actual_duration = msg.duration
            if i == len(msgs) - 1:
                actual_duration = msg.trajectory_duration
            # elif msgs[i+1].start_time < msg.start_time + msg.trajectory_duration:
            #     actual_duration = msg.trajectory_duration

            t_window = t[(t >= msg.start_time) & (t <= msg.start_time + actual_duration)]

            for t_test in t_window:
                pos, vel, acc = traj_py.compute(t_test)
                if len(td) == 0:
                    td = np.array([t_test])
                    qd = np.reshape(pos, (1,7))
                    qd_d = np.reshape(vel, (1,7))
                    qd_dd = np.reshape(acc, (1,7))
                else:
                    td = np.append(td, t_test)
                    qd = np.concatenate((qd, np.reshape(pos, (1,7))), axis=0)
                    qd_d = np.concatenate((qd_d, np.reshape(vel, (1,7))), axis=0)
                    qd_dd = np.concatenate((qd_dd, np.reshape(acc, (1,7))), axis=0)

    np.savetxt(bag_path + "joint_info_t.csv", t, delimiter=" ", fmt='%.10f')
    np.savetxt(bag_path + "joint_info_q.csv", q, delimiter=" ", fmt='%.10f')
    np.savetxt(bag_path + "joint_info_q_d.csv", q_d, delimiter=" ", fmt='%.10f')
    np.savetxt(bag_path + "joint_info_tau.csv", tau, delimiter=" ", fmt='%.10f')

    np.savetxt(bag_path + "desired_trajectory_t.csv", td, delimiter=" ", fmt='%.10f')   
    np.savetxt(bag_path + "desired_trajectory_q.csv", qd, delimiter=" ", fmt='%.10f')
    np.savetxt(bag_path + "desired_trajectory_q_d.csv", qd_d, delimiter=" ", fmt='%.10f')
    np.savetxt(bag_path + "desired_trajectory_q_dd.csv", qd_dd, delimiter=" ", fmt='%.10f')

    if len(td) > 0:
        t0 = min(t[0], td[0])
        t = t - t0
        td = td - t0

        plt.figure(figsize=(15,10))
        for i in range(7):
            plt.subplot(3,3,i+1)
            plt.plot(t, q[:,i], label="Actual")
            # plt.hold(True)
            plt.plot(td, qd[:,i], '.', label="Desired")
            plt.xlabel("Time (s)")
            plt.ylabel("Joint Position (rad)")
            plt.title("Joint " + str(i+1))
        plt.suptitle("Joint Trajectories")
        plt.savefig(bag_path + "joint_trajectory.png")

        plt.figure(figsize=(15,10))
        for i in range(7):
            plt.subplot(3,3,i+1)
            plt.plot(t, q_d[:,i], label="Actual")
            # plt.hold(True)
            plt.plot(td, qd_d[:,i], '.', label="Desired")
            plt.xlabel("Time (s)")
            plt.ylabel("Joint Velocity (rad/s)")
            plt.title("Joint " + str(i+1))
        plt.suptitle("Joint Velocities")
        plt.savefig(bag_path + "joint_velocity.png")

        plt.figure(figsize=(15,10))
        for i in range(7):
            plt.subplot(3,3,i+1)
            plt.plot(td, qd_dd[:,i], '.', label="Desired")
            plt.xlabel("Time (s)")
            plt.ylabel("Joint Acceleration (rad/s^2)")
            plt.title("Joint " + str(i+1))
        plt.suptitle("Joint Accelerations")
        plt.savefig(bag_path + "joint_acceleration.png")
