# Controller Instances

## Introduction
In `planning_wksp/`, run the following command to initialize a controller instance:
```shell
ros2 run roahm_kortex control_system --ros-args --params-file ./config.yaml
```

The configuration file is defined in `planning_wksp/config.yaml`. 
The robot hardware address is default to be `192.168.1.10` for the arm in the workspace and `192.168.1.20` for the arm in the living room scenario.

The controller updates in >= 1kHz and listens to trajectory messages (from `roahm_msgs/msg/TrajectoryMsg.msg`).
Once it receives one trajectory message, it just honestly tracks the trajectory (using the robust controller or other types of controllers) until the end of the trajectory.

## Controllers and Their Parameters

**THE FOLLOWING CONTOLLERS ARE SUPPORTED RIGHT NOW**

#### PID
Change `controller_type` to `PID` in `config.yaml`.

Parameters:
1. Kp: 7-dimensional vector for P gains
2. Ki: 7-dimensional vector for I gains
3. Kd: 7-dimensional vector for D gains

#### PID With Gravity Compensation
Change `controller_type` to `GRAV_PID` in `config.yaml`.

Parameters (same as PID parameters):
1. Kp: 7-dimensional vector for P gains
2. Ki: 7-dimensional vector for I gains
3. Kd: 7-dimensional vector for D gains

#### Robust Controller (Armour)
Change `controller_type` to `ARMOUR` in `config.yaml`.

Parameters:
1. Kr: 7-dimensional vector for D gains
2. V_max: upper bound of Lyapunov function 
3. alpha: CBF feedback
4. r_norm_threshold: when error trajectory $r$ should be considered to be 0

**THE FOLLOWING CONTOLLERS ARE IN DEVELOPMENT**

#### Passivity-based Controller
`Passivity`
Kr, Kp, Kd

#### Robust Controller (Armour Improved)
`Robust`
Kr, V_max, alpha, r_norm_threshold

#### Robust Controller (From [Ultimate Bound paper](https://ieeexplore.ieee.org/document/7525375))
`Althoff`
Kr, phi_p, phi_i, kappa_p, kappa_i, max_error

## Dual-Arm Control

To enable dual-arm control, you need to run two `control_system` instances in two terminals.
And as a result, you are going to need two different configuration files (for example, check out `config_example_robot_1.yaml` and `config_example_robot_2.yaml`).
Each of the `control_system` will have its own channel to publish `joint_info` and subscribe `trajectory` messages.

Be sure to change the configuration correctly, here are the parameters that you should look into:

 - `ip`: make sure the ip address is consistent with the corresponding ip address of the Kinova robot that you want to control.
 - `dual_arm`: turn this to `true` so that `control_system` knows to publish joint info at a specific channel.
 - `robot_id`: this is the integer to identify the robots. For example, if this is 1, then the joint info is published at `/joint_info_1`.
 - `ros_traj_topic`: this is a string that tells the robot where to subscribe for trajectory messages.

 Accordingly, in your scripts in `roahm_experiments/`, if you want to communicate with each robot on python side, make sure you subscribe the joint info or publish the trajectory messages at the correct places.