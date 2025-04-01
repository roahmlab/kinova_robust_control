#!/usr/bin/env python3
from roahm_experiments.sin_experiment import SinExperiment
import rclpy
import numpy as np

if __name__ == "__main__":
    rclpy.init()
    # amplitude and period
    amp = np.zeros(7)
    amp[0] = 1 * np.pi / 8
    amp[1] = 1 * np.pi / 8
    amp[2] = 1 * np.pi / 8
    amp[3] = 1 * np.pi / 8
    amp[4] = 1 * np.pi / 8
    amp[5] = 1 * np.pi / 8
    amp[6] = 1 * np.pi / 8

    np.random.seed(92345)
    amp = amp * ((np.random.randint(0, 2, 7) - 0.5) * 2)
    print(amp)

    T: list = [12.5, 15, 25, 15, 20, 18, 20]
    kp: float = [60, 200, 30, 60, 20, 20, 20]
    kd: float = [5, 5, 5, 5, 5, 5, 5]
    # kp: float = [20,20,20,20,20,20,20]
    # kd: float = [5, 5, 5, 5, 5, 5, 5]
    # default pos
    zero_pos: list = [0] * 7
    home_pos: list = [
        0., 0.26179939, 3.14159265, -2.2689280271795864, 0., 0.95993109,
        1.57079633
    ]
    init_pos = zero_pos
    # start running experiment
    experiment = SinExperiment(
        amp,
        T,
        kp,
        kd,
        close_loop=True,
        model_path="./models/mjcf/kinova3/gen3_gripper.mjcf",
        init_pos=init_pos,
        timeout=120,
        control_timeout=None)
    experiment.run()
