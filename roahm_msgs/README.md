# roahm_msgs

This package contains messages/service files used in other packages.

### KortexMeasurements.msg

Joint information for Kinova Gen3 Arm.

| Name     | Type                     | Description            |
| -------- | ------------------------ | ---------------------- |
| frame_id | uint32                   | index of the frame     |
| stamp    | builtin_interfaces/Time  | timestamp              |
| pos      | float32[7]               | position of each joint |
| vel      | float32[7]               | velocity of each joint |
| torque   | float32[7]               | torque of each joint   |

### TorqueControl.msg

Torque values to be applied on the arm.

| Name            | Type                   | Description                        |
| --------------- | ---------------------- | ---------------------------------- |
| frame_id        | uint32                 | index of the frame                 |
| stamp           | builtin_interfaces/Time| timestamp                          |
| torques         | float32[7]             | torque of each joint to be applied |
| is_gripper_open | bool                   | open the gripper or not            |

### TrajectoryMsg.msg

Describe a trajectory to be tracked by the controller. 
For more information, please refer to [README in roahm_trajectories/](../roahm_trajectories/README.md).

| Name               | Type         | Description                                                        |
| ------------------ | ------------ | ------------------------------------------------------------------ |
| traj_data          | float64[100] | an array storing numerical description of the trajectory           |
| start_time         | float64      | global timestamp specifying the time to start the trajectory       |
| trajectory_duration| float64      | time in seconds specifying the total duration that the trajectory is defined |
| duration           | float64      | time in seconds specifying the duration that the trajectory will be executed |
| dof                | int32        | degree of freedom of the robot (dimensions of the robot)           |
| trajectory_type    | int32        | the type of the trajectories to track                              |
| is_gripper_open    | bool         | open the gripper or not (not used)                                 |
| reset              | bool         | currently not used                                                           |

<!-- ### Debug.msg

| Name        | Type            | Description                                            |
| ----------- | --------------- | ------------------------------------------------------ |
| header      | std_msgs/Header | header type containing id, timestamps...               |
| pos_d       | float32[7]      | desired position for each joint                        |
| vel_d       | float32[7]      | desired velocity for each joint                        |
| torque_d    | float32[7]      | desired torque for each joint                          |
| pos_curr    | float32[7]      | measured position for each joint                       |
| vel_curr    | float32[7]      | measured velocity for each joint                       |
| torque_curr | float32[7]      | measured torque for each joint                         |
| torque_calc | float32[7]      | calculated torque for each joint by dynamics algorithm |
| e           | float32[7]      | position error computed by dynamics algorithm          |
| e_d         | float32[7]      | velocity error computed by dynamics algorithm          | -->
