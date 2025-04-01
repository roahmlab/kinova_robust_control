#!/usr/bin/env python3
from roahm_experiments.base_experiment import BaseExperiment
import roahm_dynamics_py
import numpy as np
from roahm_experiments.services import KortexSrvs
from typing import Optional
import time


class SinExperiment(BaseExperiment):
    """
    generate a sinusoidal trajectory and use pd control to get torque
    """

    def __init__(self,
                 amp: list,
                 T: list,
                 kp: list,
                 kd: list,
                 init_pos: list = [0] * 7,
                 data_prefix: str = "",
                 pub_topic: str = "roahm_control",
                 model_path: str = "models/mjcf/gen3_nof.mjcf",
                 close_loop: bool = False,
                 timeout: Optional[int] = None,
                 control_timeout: Optional[int] = None):
        """
        initializer
        :param list amp: amplitude of each joint
        :param list T: period of the joint
        :param float kp: proportional gain in pd
        :param float kd: differential gain in pd
        :param str data_prefix: filename prefix to save, default class name
        :param str pub_topic: topic to publish dynamics
        :param bool close_loop: whether to do closed loop control
        """
        assert len(amp) == 7
        assert len(T) == 7
        # init base experiment
        if data_prefix == "":
            data_prefix = self.__class__.__name__
        super().__init__(data_prefix,
                         timeout=timeout,
                         control_timeout=control_timeout)
        # store values
        self.amp: np.ndarray = np.array(amp)
        self.omega: np.ndarray = 2 * np.pi / np.array(T)
        self.kp: np.ndarray = np.array(kp)
        self.kd: np.ndarray = np.array(kd)
        self.init_pos: np.ndarray = np.array(init_pos)
        self.close_loop = close_loop

        # rnea solver
        self.solver = roahm_dynamics_py.SRNEA(model_path)

        self.service = KortexSrvs()
        self.service.goto((self.init_pos + self.amp).astype(np.float32))

        self.service.start_torque_control()
        self.t0 = time.time()

    def _callback(self, jt_msg):
        """
        callback function for joint_info topic
        update data and publish back the control command
        :param joints jt_msgs:
        """
        super()._callback(jt_msg)

    def _get_torque(self, jt_msg) -> list:
        """
        calculate torque using RNEA and pd control
        :param joints jt_msg: joint_info message from kinova
        :return list: list of torque
        """
        q: np.ndarray = np.array(jt_msg.pos)
        qd: np.ndarray = np.array(jt_msg.vel)
        # desired state
        theta: float = (time.time() - self.t0) * self.omega
        q_des: np.ndarray = self.amp * np.cos(theta) + self.init_pos
        qd_des: np.ndarray = -self.amp * self.omega * np.sin(theta)
        qdd_des: np.ndarray = (-self.amp * self.omega * self.omega *
                               np.cos(theta))
        torque_cal: np.ndarray = self.solver.solve(q_des, qd_des, qd_des,
                                                   qdd_des)
        # close loop control
        if self.close_loop and (self.control_timeout is None or
                                time.time() - self.t0 < self.control_timeout):
            # error
            e: np.ndarray = q_des - q
            ed: np.ndarray = qd_des - qd
            for i in [0, 2, 4, 6]:
                while e[i] < -np.pi:
                    e[i] += 2 * np.pi

                while e[i] > np.pi:
                    e[i] -= 2 * np.pi

            # add pd term
            torque_cal += self.kp * e + self.kd * ed

        # truncate output to prevent overflow
        torque_cal[:4] = torque_cal[:4].clip(-35, 35)
        torque_cal[4:] = torque_cal[4:].clip(-8.9, 8.9)
        return torque_cal.tolist()
