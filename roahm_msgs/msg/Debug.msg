# Debug Information for Visualization
float64 time

# Joint State
roahm_msgs/KortexMeasurements joint_states

# trajectory
roahm_msgs/TrajectoryMsg traj

# torque applied
roahm_msgs/TorqueControl control

# Dynamics Algorithm computes e, e_d while computing required torque
float32[7] e
float32[7] ed

float32[7] q_des
float32[7] qd_des
float32[7] qdd_des
