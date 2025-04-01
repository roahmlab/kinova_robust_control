#!/usr/bin/env python3
"""
Pick a certain start and set of torques and apply on the robot at a fixed frequency
:ref: http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient
"""
import sys
from roahm_msgs.msg import TorqueControl, KortexMeasurements
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod
import numpy as np
import time
from typing import Optional
from threading import Lock


class BaseExperiment(ABC):
    """
    Experiment Base class
    """

    def __init__(self,
                 data_prefix: str = "",
                 pub_topic: str = "roahm_control",
                 timeout: Optional[int] = None,
                 control_timeout: Optional[int] = None):
        """
        constructor
        :param str data_prefix: filename prefix to save, default class name
        :param str pub_topic: topic to publish dynamics
        """
        self.node: Node = Node(self.__class__.__name__)
        self.publisher = self.node.create_publisher(TorqueControl, 'dynamics', 10)
        self.data_prefix = data_prefix
        self.timeout = timeout
        self.control_timeout = control_timeout
        if self.data_prefix == "":
            self.data_prefix = self.__class__.__name__
        # data to retrieve and save, can add more in derived class
        # int frame: frame id
        # float t: frame timestamp
        # np.ndarray feedback_pos: feedback position
        # np.ndarray feedback_vel: feedback velocity
        # np.ndarray torque: torque applied
        self.data: dict = {
            "frame": [],
            "frame_ts": [],
            "feedback_pos": [],
            "feedback_vel": [],
            "feedback_torque": [],
            "torque": [],
        }
        self.stop: bool = False
        self.save: bool = False
        self.mtx = Lock()
        self.pub_topic = pub_topic
        # may be refreshed
        self.t0 = time.time()

    def run(self):
        """
        run the experiment
        """
        # register callback and start experiment
        sub = self.node.create_subscription(
            KortexMeasurements,
            'joint_info',
            self._callback,
            10)

        msg = TorqueControl()
        msg.frame_id = 0
        msg.torque = np.zeros(7, dtype=np.float32)
        self.publisher.publish(msg)

        # wait for shutdown
        import threading
        input_thread = threading.Thread(target=self._check_input)
        input_thread.start()
        with self.mtx:
            stop = self.stop
        while (self.timeout is None
               or time.time() - self.t0 < self.timeout) and not stop:
            rclpy.spin_once(self.node)
            with self.mtx:
                stop = self.stop

        # shutdown node and save data
        print("Complete, press q to quit or s to save")
        input_thread.join()
        if self.save:
            self._save()

    def _check_input(self):
        while True:
            cli_input: str = input("")
            print(cli_input)
            if cli_input in ["q", "s"]: break

        # set values
        with self.mtx:
            self.stop = True
            if cli_input == "s":
                self.save = True

    def _callback(self, jt_msg):
        """
        callback function for joint_info topic
        update data and publish back the control command
        :param joints jt_msgs:
        """
        # send torque command
        msg = TorqueControl()
        msg.frame_id = jt_msg.frame_id + 1
        msg.torque = self._get_torque(jt_msg)
        self.publisher.publish(msg)

        # record data
        if self.control_timeout is None or time.time(
        ) - self.t0 > self.control_timeout:
            curr_time = time.time()
            self.data["frame"].append(jt_msg.frame_id)
            self.data["frame_ts"].append(curr_time)
            self.data["feedback_pos"].append(np.array(jt_msg.pos))
            self.data["feedback_vel"].append(np.array(jt_msg.vel))
            self.data["feedback_torque"].append(np.array(jt_msg.torque))
            self.data["torque"].append(np.array(msg.torque))

    @abstractmethod
    def _get_torque(self, jt_msg) -> list:
        """
        calculate torque based on joint message
        :abstract: to be implemented in child class
        """
        return []

    def _save(self) -> None:
        """
        save the result to file
        """
        # convert to np array
        for key in self.data.keys():
            self.data[key] = np.array(self.data[key])
        # save file
        np.savez_compressed(
            "{prefix}_{time}".format(prefix=self.data_prefix,
                                     time=str(round(time.time()))),
            **self.data)
