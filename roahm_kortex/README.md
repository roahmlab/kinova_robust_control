# Controller Instances

## Introduction
In `/workspaces/kinova_robust_control_docker/`, run the following command to initialize a controller instance:
```shell
ros2 run roahm_kortex control_system --ros-args --params-file ./config.yaml
```

The configuration file should be defined in `/workspaces/kinova_robust_control_docker/config.yaml`. 
We have provided an example configuration file [here](../config_example.yaml).
The robot hardware address is default to be `192.168.1.10`.

The controller updates in >= 1kHz and listens to trajectory messages in the format of `roahm_msgs/msg/TrajectoryMsg.msg` and at channel defined in `ros_traj_topic` field in the configuration file.
Once it receives one trajectory message, it just honestly tracks the trajectory (using the robust controller or other types of controllers) until the end of the trajectory.

## Parameters In `config.yaml` under `/control_system`

### System Information

| Parameter     | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `has_init_pos` | `true` if the robot should move to an initial position at controller startup. Useful when repeating experiments. |
| `init_pos`     | 7-dimensional initial joint position.                                       |

### Robot Information

| Parameter            | Description                                                                                  |
|----------------------|----------------------------------------------------------------------------------------------|
| `ip`                 | String representing the robot's hardware address. Defaults to `"192.168.1.10"`.             |
| `model_path`         | Path (relative to workspace) to the robot URDF file.                                         |
| `ros_traj_topic`     | Topic name to subscribe for trajectory messages. Use different names for controlling multiple arms. |
| `friction`           | 7-dimensional static friction coefficient for each motor.                                    |
| `damping`            | 7-dimensional damping friction coefficient for each motor.                                   |
| `transmissionInertia` | 7-dimensional transmission inertia (armature) of each motor.                                |
| `offset`             | 7-dimensional frictional offset for each motor.                                              |
| `*_eps`              | Ratio of model parameter uncertainty. For example, `0.05` means 5% uncertainty.              |


### Controllers and Their Parameters

**THE FOLLOWING CONTOLLERS ARE SUPPORTED RIGHT NOW**

#### PID
Change `controller_type` to `PID` in `config.yaml`.

| Parameter | Description |
|-----------|-------------|
| `Kp` | 7-dimensional vector for P gains |
| `Ki` | 7-dimensional vector for I gains |
| `Kd` | 7-dimensional vector for D gains |

#### PID With Gravity Compensation
Change `controller_type` to `GRAV_PID` in `config.yaml`.

| Parameter | Description |
|-----------|-------------|
| `Kp` | 7-dimensional vector for P gains |
| `Ki` | 7-dimensional vector for I gains |
| `Kd` | 7-dimensional vector for D gains |

#### Robust Controller (Armour)
Change `controller_type` to `ARMOUR` in `config.yaml`.

| Parameter | Description |
|-----------|-------------|
| `Kr` | 7-dimensional vector for $K_r$ used in passivity-based control law. higher value yields more aggressive behavior for better tracking performance |
| `V_max` | Upper bound of Lyapunov function. lower value yields more aggressive behavior for better tracking performance |
| `alpha` | CBF feedback, higher value yields more aggressive behavior for better tracking performance |
| `r_norm_threshold` | When error trajectory $r$ should be considered to be 0 |

**THE FOLLOWING CONTOLLERS ARE IN DEVELOPMENT**

#### Nominal Passivity-based Controller
`Passivity`

#### Classical Adaptive Controller [1]
`Adaptive`

#### Robust Controller [2]
`Althoff`

## Dual-Arm Control

To enable dual-arm control, you need to run two `control_system` instances in two terminals.
And as a result, you are going to need two different configuration files (for example, check out `config_example_robot_1.yaml` and `config_example_robot_2.yaml`).
Each of the `control_system` will have its own channel to publish `joint_info` and subscribe `trajectory` messages.

Be sure to change the configuration correctly, here are the parameters that you should look into:

| Parameter | Description |
|-----------|-------------|
| `ip` | Make sure the ip address is consistent with the corresponding ip address of the Kinova robot that you want to control |
| `dual_arm` | Turn this to `true` so that `control_system` knows to publish joint info at a specific channel |
| `robot_id` | This is the integer to identify the robots. For example, if this is 1, then the joint info is published at `/joint_info_1` |

 Similarly, you need to change `ros_traj_topic` to let the controller to know where to subscribe the correct trajectory messages.
 Accordingly, in your scripts in `roahm_experiments/`, if you want to communicate with each robot on python side, make sure you subscribe the joint info or publish the trajectory messages at the correct places:

 ```python
# trajectory publisher
self.traj_pub_1 = self.create_publisher(TrajectoryMsg, "/trajectory_1", 10) # be consistent with `ros_traj_topic` in the corresponding configuration file
self.traj_pub_2 = self.create_publisher(TrajectoryMsg, "/trajectory_2", 10) # be consistent with `ros_traj_topic` in the corresponding configuration file

# joint measurement
self.joint_info_sub_1 = self.create_subscription(KortexMeasurements, "/joint_info_1", self._joint_info_1_callback, 10) # be consistent with `robot_id` in the corresponding configuration file
self.joint_info_sub_2 = self.create_subscription(KortexMeasurements, "/joint_info_2", self._joint_info_2_callback, 10) # be consistent with `robot_id` in the corresponding configuration file
 ```

 For more information, please refer to [test_trajectories_dual_arm.py](../roahm_experiments/scripts/test_trajectories_dual_arm.py).

 ## References

 [1] Slotine, Jean-Jacques E., and Weiping Li. "On the adaptive control of robot manipulators." The international journal of robotics research 6.3 (1987): 49-59.

 [2] Giusti, Andrea, and Matthias Althoff. "Ultimate robust performance control of rigid robot manipulators using interval arithmetic." 2016 American Control Conference (ACC). IEEE, 2016.