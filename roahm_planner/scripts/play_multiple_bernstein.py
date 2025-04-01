#!/usr/bin/env python3
from roahm_msgs.msg import TrajectoryMsg
import rclpy
from rclpy.node import Node
from time import sleep
import time
import numpy as np
import math
import sys

# trajs_path: list = [
#     "../data/traj/demo1118/traj1.npz", "../data/traj/demo1118/traj2.npz",
#     "../data/traj/demo1118/traj3.npz"
# ]

trajs_path: list = [
    "../data/traj/demo1118/traj_dumbbell1.npz",
    "../data/traj/demo1118/traj_dumbbell2.npz",
    "../data/traj/demo1118/traj_dumbbell3.npz"
]

# settings
time_wait: float = 4.  # time to sync initial position
time_advance: float = 0.5  # time to plan prior to execution

if __name__ == "__main__":
    rclpy.init()
    node = Node("traj_player")
    publisher = node.create_publisher(TrajectoryMsg, "trajectory", 10)

    msg = TrajectoryMsg()
    msg.reset = False
    msg.start_time = 0.
    for tp in trajs_path:
        # start sending message
        data = np.load(tp)
        num_data = max([
            data['k'].shape[0], data['qdd0'].shape[0], data['qd0'].shape[0],
            data['q0'].shape[0]
        ])
        # NOTE: send the first position to make sure the tracking error
        #       lies within the ultimate bound
        msg.duration = time_wait + time_advance
        msg.pos = data["q0"][0]
        msg.acc = np.zeros_like(data["qdd0"][0])
        msg.vel = np.zeros_like(data["qd0"][0])
        msg.k = np.zeros_like(data["k"][0])
        msg.is_gripper_open = True
        publisher.publish(msg)
        print(msg)
        sleep(time_wait)
        msg.duration = 0.5
        msg.start_time += time_wait + time_advance
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
            publisher.publish(msg)
            print(msg)
            msg.start_time += dt

            sleep(dt - (time.time() - start))

    # msg.acc = np.zeros_like(msg.acc)
    # msg.vel = np.zeros_like(msg.vel)
    # msg.k = np.zeros_like(msg.k)
    # msg.duration = math.inf
    # msg.is_gripper_open = True
    # publisher.publish(msg)
