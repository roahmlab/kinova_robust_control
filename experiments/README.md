# experiments

This folder contains example scripts for controlling Kinova robots using our controller interface. 
The scripts are organized into two main categories:

1. **Service Examples**: Basic robot control using default Kortex services for tasks like moving to positions and picking up objects.
2. **Controller Examples**: Advanced examples for tracking a series of continuous trajectories.

Each example requires specific ros2 controller instances to be running (`ros_kortex_system` or `control_system`) with appropriate configuration files.

## service examples (run with `ros_kortex_system`)

Use kinova official service handler to go to somewhere or open/close the gripper.
Run the following ros2 command in a separate terminal before running any of the scripts.
```shell
ros2 run kortex ros_kortex_system --ros-args --params-file ./config.yaml
```

### test_services.py
This script contains the most basic example of setting up a kortex service requester and go to a predefined home position.

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments test_services.py
```

### pick_up_example.py
This script contains a simple example of picking up an object at a predefined location.
It also contains an inverse kinematics solver from `pinocchio`, implemented in `pinocchio_ik_solver.py`.

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments pick_up_example.py
```

## controller examples (run with `control_system`)

Use robust controller or other controllers to track a continuous trajectory.
Run the following ros2 command in a separate terminal before running any of the scripts.
```shell
ros2 run kortex control_system --ros-args --params-file ./config.yaml
```

### test_trajectories.py
This script contains **the most basic example** of **sending trajectory messages**.
We formulate a Armour trajectory message that asks the arm to move forward 0.05 radian slowly from its current position and send this message periodically.
Since the duration of the trajectory is **smaller than** the time gap bewteen two messages, the arm should move and stop repetitively.

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments test_trajectories.py
```

### test_trajectories_dual_arm.py
This script contains **the most basic example** of **sending two different trajectory messages to two robots at the same time**.
We formulate a Armour trajectory message that asks the arm to move forward 0.05 radian slowly from its current position and send this message periodically.
Since the duration of the trajectory is **smaller than** the time gap bewteen two messages, the arm should move and stop repetitively.

Make sure you launch two `control_system` for each of the robot properly by providing each of them with a proper configuration file (Check out [config_example_robot_1](../config_example_robot_1.yaml) and [config_example_robot_2](../config_example_robot_2.yaml)).

Launch the script in `/workspaces/kinova_robust_control_docker` using:
```shell
ros2 run experiments test_trajectories_dual_arm.py
```

<!-- ### test_armtd_trajectories.py
This script contains the most basic example of sending ARMTD trajectory messages.

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments test_armtd_trajectories.py
``` -->

### test_multiple_continuous_trajectories.py
We formulate a Bezier trajectory message that asks the arm to move forward 0.1 radian slowly from its current position and send this message periodically.
Since the duration of the trajectory is **equal to** the time gap bewteen two messages, every adjacent trajectories are considered to be "connected" so the arm should move smoothly without stopping.

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments test_multiple_continuous_trajectories.py
```

### test_receding_horizon_trajectories.py
This is the most complicated example script here.
We formulate a Bezier trajectory message that asks the arm to move forward 0.35 radian slowly from its current position and send this message periodically.
Note that the first several trajectories are receding horizon trajectories, which means one is connected to the previous one in the middle.
The arm should move smoothly.
The last trajectory is an invalid trajectory, so you should see the controller instance pops up a warning and ignore this one. 
It will continue to execute the previous trajectory and stop safely eventually. 

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments test_receding_horizon_trajectories.py
```

<!-- ### test_receding_horizon_trajectories_armtd.py
Similar tests but for ARMTD trajectories.

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments test_receding_horizon_trajectories_armtd.py
``` -->

<!-- ### pick_up_and_down_example.py
This script contains a simple example of picking up and placing down an object at a predefined location, using `control_system` (torque control mode).
It requires [RAPTOR](https://github.com/roahmlab/RAPTOR) for a pybind version of inverse kinematics solver.

Launch the script in `/workspaces/kinova_robust_control_docker/` using:
```shell
ros2 run experiments pick_up_and_down_example.py
``` -->