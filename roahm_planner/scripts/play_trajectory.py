#!/usr/bin/env python3
from roahm_msgs.msg import TrajectoryMsg
import rclpy
from rclpy.node import Node
from time import sleep
import numpy as np
import math

# settings
data_path = "./data/traj/robust_traj2.npz"
time_wait: float = 2.  # time to sync initial position
time_advance: float = 0.5  # time to plan prior to execution

if __name__ == "__main__":
    rclpy.init()
    node = Node("traj_player")
    publisher = node.create_publisher(TrajectoryMsg, "trajectory", 10)
    data = np.load(data_path)
    num_data = max(
        [data['q'].shape[0], data['qd'].shape[0], data['qdd'].shape[0]])
    msg = TrajectoryMsg()
    msg.reset = False
    # NOTE: send the first position to make sure the tracking error
    #       lies within the ultimate bound
    msg.duration = time_wait + time_advance
    msg.acc = data["qdd"][0]
    msg.vel = data["qd"][0]
    msg.pos = data["q"][0]
    msg.is_gripper_open = not data["is_gripper_open"][0]
    msg.start_time = 0.
    publisher.publish(msg)
    print(msg)
    sleep(time_wait)

    # start sending message
    msg.duration = 0.5
    for i in range(num_data):
        start = time.time()
        msg.acc = data["qdd"][i]
        msg.vel = data["qd"][i]
        msg.pos = data["q"][i]
        # msg.is_gripper_open = not data["grasp"][i]
        msg.is_gripper_open = not data["is_gripper_open"][i]
        msg.start_time = 0.5 * i + time_wait + time_advance
        publisher.publish(msg)
        print(msg)
        sleep(0.5 - (time.time() - start))

    msg.acc = np.zeros(7)
    msg.vel = np.zeros(7)
    msg.duration = math.inf
    msg.is_gripper_open = True
    publisher.publish(msg)
