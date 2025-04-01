#!/usr/bin/env python3
from roahm_msgs.msg import TrajectoryMsg
import rclpy
from rclpy.node import Node
from time import sleep
import time
import numpy as np
import math
import sys

# settings
time_wait: float = 2.  # time to sync initial position
time_advance: float = 0.5  # time to plan prior to execution


def bernstein_q_des(q0, qd0, qdd0, k, t):

    def pow(x, y):
        return x**y

    B0 = -pow(t - 1, 5)
    B1 = 5 * t * pow(t - 1, 4)
    B2 = -10 * pow(t, 2) * pow(t - 1, 3)
    B3 = 10 * pow(t, 3) * pow(t - 1, 2)
    B4 = -5 * pow(t, 4) * (t - 1)
    B5 = pow(t, 5)
    beta0 = q0
    beta1 = q0 + qd0 / 5
    beta2 = q0 + (2 * qd0) / 5 + qdd0 / 20
    beta3 = q0 + k
    beta4 = q0 + k
    beta5 = q0 + k
    return (B0 * beta0 + B1 * beta1 + B2 * beta2 + B3 * beta3 + B4 * beta4 +
            B5 * beta5)


if __name__ == "__main__":
    rclpy.init()
    node = Node("traj_player")
    node.declare_parameter('data_path',
                           '../data/traj/demo1118/force_closure_success.npz')
    publisher = node.create_publisher(TrajectoryMsg, "trajectory", 10)
    data_path = node.get_parameter(
        'data_path').get_parameter_value().string_value
    data = np.load(data_path)
    num_data = max([
        data['k'].shape[0], data['qdd0'].shape[0], data['qd0'].shape[0],
        data['q0'].shape[0]
    ])
    msg = TrajectoryMsg()
    msg.reset = False
    msg.is_gripper_open = False

    time.sleep(5)
    # NOTE: send the first position to make sure the tracking error
    #       lies within the ultimate bound
    msg.duration = time_wait + time_advance
    msg.pos = data["q0"][0]
    msg.acc = np.zeros_like(data["qdd0"][0])
    msg.vel = np.zeros_like(data["qd0"][0])
    msg.k = np.zeros_like(data["k"][0])
    msg.is_gripper_open = False
    msg.start_time = 0.
    publisher.publish(msg)
    print(msg)
    sleep(time_wait)

    # start sending message
    msg.duration = 0.5
    for i in range(num_data):
        start = time.time()
        if i != num_data - 1:
            dt = 0.5
        else:
            dt = 1.
        msg.acc = data["qdd0"][i]
        msg.vel = data["qd0"][i]
        msg.pos = data["q0"][i]
        msg.k = data["k"][i]
        msg.duration = dt
        msg.start_time = 0.5 * i + time_wait + time_advance
        publisher.publish(msg)
        print(msg)
        sleep(dt - (time.time() - start))

    msg.acc = np.zeros_like(msg.acc)
    msg.vel = np.zeros_like(msg.vel)
    msg.k = np.zeros_like(msg.k)
    msg.pos = bernstein_q_des(data["q0"][-1], data["qd0"][-1],
                              data["qdd0"][-1], data["k"][-1], 1)
    msg.duration = math.inf
    msg.is_gripper_open = False
    publisher.publish(msg)
