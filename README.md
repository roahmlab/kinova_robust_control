# kinova_robust_control

## Install

In `planning_wksp/`, run the following command to compile the code:
```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then source the generated setup files by running 
```shell
source install/setup.bash
```
Note that you need to do this **EVERY TIME** you open a new terminal!

## Motivation

For this repo, there are two groups of target audience.

### I don't care about the controller or the trajectory, just go to somewhere for me

Simply initialize a kinova robot official control instance in one terminal that specifically corresponds to a robot's physcal address:
```shell
ros2 run roahm_kortex ros_kortex_system --ros-args --params-file ./config.yaml
```
Specify proper parameters in `planning_wksp/config.yaml` (under `/ros_kortex_system`):
1. `ip`: which robot do you want to control? `192.168.1.10` for the arm in the workspace and `192.168.1.20` for the arm in the living room scenario.

This gives you a kinova robot control instance that is always listening to you and waiting for your service request.

### I don't care about the controller, just track a trajectory for me

Simply initialize a controller instance in one terminal that specifically corresponds to a robot's physcal address:
```shell
ros2 run roahm_kortex control_system --ros-args --params-file ./config.yaml
```
Specify proper parameters in `planning_wksp/config.yaml` (under `/control_system`):
1. `ip`: which robot do you want to control? `192.168.1.10` for the arm in the workspace and `192.168.1.20` for the arm in the living room scenario.
2. `controller_type`: which controller do you want to use? Currently only `ARMOUR` and `PID` are supported. And there are corresponding controller parameters that you can play with.

This gives you a controller instance that is always listening to you. 
Send a proper trajectory message to `/trajectory` channel and the controller will receive that and honestly track that!

### I want to play with the controllers

You can implement your own controller in `roahm_dynamics` for testing.
Simply add your new controller implementation and add access to that for the controller instance in `roahm_kortex/src/control_system.cpp`.

## Getting Started

Read the following instructions in sequence:
1. Read [roahm_trajectories/README.md](roahm_trajectories/README.md) to learn how to formulate a proper trajectory message. The controller will not execute the trajectory if the trajectory is invalid.
2. Read [roahm_kortex/README.md](roahm_kortex/README.md) to learn more about the controller instance and related parameters.
3. Read [roahm_experiments/README.md](roahm_experiments/README.md) for examples that send trajectory messages to the controller instance.

Other stuff
1. Read [roahm_msgs/README.md](roahm_msgs/README.md) for definitions of all ROS messages.

## Acknowlgement

The repository relies on [KINOVA® KORTEX™ API Reference](https://github.com/Kinovarobotics/kortex) developed and maintained by [Kinova Robotics](https://www.kinovarobotics.com/). 
We gratefully acknowledge their work and contribution to the open-source robotics community.

## Authors

[Bohao Zhang](https://cfather.github.io/) (jimzhang@umich.edu): **Current maintainer**, Robust controller in C++.

Jonathan Michaux (jmichaux@umich.edu): Robust controller theory developer.

Patrick D. Holmes (pdholmes@umich.edu): Robust controller theory developer.

Che Chen (cctom@umich.edu): Original creator and maintainer of the repository.

Zichang Zhou (zhouzichang1234@gmail.com): Implemented other controller comparisons.

## Rules
If you have any questions or suggestions, please raise them in [Issues](https://github.com/roahmlab/kinova_robust_control/issues).
We will get back to you as soon as possible.