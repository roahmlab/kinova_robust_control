# roahm_trajectories

This folder contains the implementation of different types of trajectories

## Trajectories

In this folder, we define the following types of trajectories.

### ARMTD_TRAJ (0x3001)
The trajectory used in ARMTD (Equation 2, https://arxiv.org/abs/2002.01591)
It consists of two pieces of trajectories with constant acceleration. 
One for going forward and another one for braking.

It requires initial position, velocity of each joint.
It also requires a constant for the acceleration of the first piece of trajectory, denoted as `k`.
It finally requires a braking time, which should be strictly smaller than the duration of the trajectory, and is recommended to be the half of the duration.
The braking time means when the robot starts to brake, after the first piece of the trajectory.
The acceleration of the second piece of the trajectory is just another constant that makes the robot stop (decelerate to 0 velocity).

Number of parameters: 3 * num_joints + 1 (3*7+1 = 22 for Kinova-gen3)

How parameters are stored in the vector:
```
[q0, qd0, k, ... repeated for each joint ...
 t_brake]
```

Note that ARMTD trajectory is **not 2nd-order continuous**, which could make the robust controller **struggle to track**.
You should consider tracking this trajectory using a PID controller or PID controller with gravity compensation.

### ARMOUR_TRAJ (0x3002)
The trajectory used in ARMOUR (Section IX.A, https://arxiv.org/abs/2301.13308)

A degree-5 Bezier curve that satisfies the following properties:
1. The position starts at a provided parameter q0.
2. The velocity starts at a provided parameter qd0.
3. The acceleration starts at a provided parameter qdd0.
4. The position ends at a provided parameter q1.
5. The velocity ends at 0.
6. The acceleration ends at 0.

This is basically a trajectory that tells the robot to **reach and stop at** a particular position starting from a certain initial condition.
This is the **most commonly used** trajectory in most applications.

Number of parameters: 4 * num_joints (4*7 = 28 for Kinova-gen3)

How parameters are stored in the vector:
```
[q0, qd0, qdd0, q_end, ... repeated for each joint ...]
```

### FOURIER_TRAJ (0x3003)
The trajectory by representing acceleration as Fourier series, mainly used in system identification
```
qdd(t) = a_0 + sum_{i=1}^{degree} a_i * cos(i*w*t) + b_i * sin(i*w*t)
qd(t) = qd0 + int_{0}^{t} qdd(t) dt
q(t) = q0 + int_{0}^{t} qd(t) dt
```
Number of parameters: (2 * degree + 3) * num_joints + 1 ((2*4+3)*7+1 = 78 for Kinova-gen3 if degree = 4)

How parameters are stored in the vector:
```
[a_0, a_i, b_i, ... repeated for each joint ...
q0, ... repeated for each joint ...
qd0, ... repeated for each joint ...
w]
```

### FIFTH_ORDER_BEZIER_TRAJ (0x3004)
The trajectory is a more generalized version with respect to ARMOUR_TRAJ, which is also a fifth-order Bezier curve.
The user only needs to provide the initial position, velocity, acceleration, and the end position, velocity, acceleration.
ARMOUR_TRAJ describes a special case that the end velocity and acceleration are 0.

Number of parameters: 6 * num_joints (6*7 = 42 for Kinova-gen3)

How parameters are stored in the vector:
```
[q0, qd0, qdd0, q_end, qd_end, qdd_end, ... repeated for each joint ...]
```

## Trajectory Manager

There are certain rules that you should follow to send different types of trajectories to execute.
That's why we implemented a trajectory manager to make sure the trajectory messages are formulated properly.
The trajectory manager is running together within the controller instance and keeps listening to any new trajectory message.
It has a queue that stores all received trajectory messages and deal with them in sequence.
If it receives a trajectory message that contains an invalid trajectory (we will explain what "invalid" means later), it will pop out a warning and IGNORE this trajectory.
It is **YOUR RESPONSIBILITY** to make sure the rules are strictly followed.

### Important Concepts Related to Time
There are several important concepts here in terms of definition of a trajectory:
1. All time-related variables in the following discussion is in the unit of **SECONDS**.
2. `start_time`: The **global** time frame that the trajectory will be started. Check the concept of [Unix Time](https://en.wikipedia.org/wiki/Unix_time).
3. `trajectory_duration`: The total duration that the trajectory is defined. You have to make sure that the trajectory is mathematically computable on the interval of `[0, trajectory_duration]`. For example, regular Bezier curves are defined only on [0,1]. It returns meaningless values beyond that range.
4. `duration`: The duration that the trajectory will be played. In other words, the robot should either stop safeely or switch to the next valid trajectory at `start_time + duration`. `duration` does not have to be equal to `trajectory_duration`. For [receding horizon](https://en.wikipedia.org/wiki/Model_predictive_control) trajectories, the duration is always smaller than `trajectory_duration`. However, `duration` can never be larger than `trajectory_duration`.
5. The time that will be input to class `Trajectory` and class `TrajectoryManager` will always be in **global** time frame.

### Trajectory Message Rules
1. The start time should always be later than the time when the trajectory message is sent. In other words, you can not ask the robot to execute a trajectory that already starts.
2. The trajectory should never start eariler than any other previous trajectories (that have been received by the trajectory manager prior to this trajectory).
3. `duration` should never be larger than `trajectory_duration`.
4. For two receding horizon trajectories, they should be 2nd-order continuous. For example, traj1 with `trajectory_duration = 2`, `duration = 1`, `start_time = 0` and traj2 with `trajectory_duration = 2`, `duration = 1`, `start_time = 1` are two valid receding horizon trajectories. traj2 is starting in the middle of traj1. As a result, the position, the velocity, and the acceleration are required to be equal for traj1 in the middle and traj2 at the beginning, so that they are connected not only on position, but also on velocity and acceleration. **Note that the condition on acceleration can be relaxed if you are working with ARMTD trajectories!**
5. If there is only one trajectory left in the queue of the trajectory manager and there is no new trajectory messages received. The trajectory manager will ignore `duration` and play the trajectory until its end (`trajectory_duration`). So it is a good habit to always send a trajectory **that has 0 velocity and 0 acceleration at the end**, so that the robot stops smoothly.

### Two Main Scenarios We Care About

#### Execute And Stop
The robot will execute one trajectory and stops for some time. 
And then the robot will execute the next trajectory and stop after that as well. 
In this instance:
1. `duration` should be strictly equal to `trajectory_duration`, since we are playing the entire trajectory each time.
2. `start_time_current + duration_current` is strictly smaller than or equal to `start_time_next`. In other words, there should be a small period between two consecutive trajectories that the robot will just stop there.
3. It is recommended that the velocity and the acceleration at the end of each trajectory (at `start_time + trajectory_duration`) should be strictly 0, or at least small, so that the robot can brake smoothly. This is healthier for the hardware.

#### Receding Horizon With Braking Maneuver
The robot will continuously execute a series of trajectories in a row, while each trajectory contains a period of braking maneuver.
In case that the planner fails to find out a feasible trajectory and send a message in time, the robot will just execute the braking maneuver of the previous trajectory and come to a safe stop.
1. `duration` should be strictly smaller than `trajectory_duration`.
2. `start_time_current + duration_current` should be strictly equal to `start_time_next`, so that the consecutive trajectories continue with each other in time.
3. The position, the velocity and the acceleration of the current trajectory at `start_time_current + duration_current` should be strictly equal to the position, the velocity and the acceleration of the next trajectory at `start_time_next`, so that the consecutive trajectories are continuous on up to 2nd order derivatives and the controller could transfer to the next trajectory smoothly.
4. The velocity and the acceleration at the end of each trajectory should be strictly 0.
5. If the planner, for example, failes to solve for a feasible trajectory in time and still sends a trajectory message, the trajectory manager will be able to detect that and ignore the message and perform a safe stop, as expected.

## Programming Manual

Here are some helpful tips when you program with things in roahm_trajectories:

1. `trajectory_type` should only be selected in the macros defined in `roahm_trajectories/include/roahm_trajectories/Trajectories.hpp`. Otherwise, the trajectory manager will not recognize the trajectory and ignore it.
2. The python version of trajectory type macros is defined in `roahm_trajectories/roahm_trajectories/TrajectoryMacros.py`. Make sure you `from roahm_trajectories import TrajectoryMacros` if you are formulating trajectory messages in python.
3. `traj_data` is an array storing the information of the trajectory. The size of it is hardcoded as `TRAJECTORY_DATA_SIZE` defined in `roahm_trajectories/include/roahm_trajectories/Trajectories.hpp` for now. This number is also required to be equal to the size of `traj_data` defined in `roahm_msgs/msg/TrajectoryMsg.msg`.
4. Take a quick look at `roahm_experiments/scripts/trajectory_helper.py`, which provide helper functions to formulate trajectory messages more easily.