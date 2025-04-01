#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

namespace_name: str = "force_closure_demo"


def generate_launch_description():
    ip_launch_arg = DeclareLaunchArgument(
        "ip", default_value=TextSubstitution(text="192.168.1.11"))
    data_launch_arg = DeclareLaunchArgument(
        "data_path",
        default_value=TextSubstitution(
            text="../data/traj/demo1118/force_closure_fail.npz"))
    return LaunchDescription([
        ip_launch_arg,
        data_launch_arg,
        Node(
            package='roahm_kortex',
            executable='robust_control_system',
            name='robust_control_system',
            parameters=[{
                "model_path":
                "./models/mjcf/kinova3/gen3_gripper_cube.mjcf",
                "ip":
                LaunchConfiguration("ip"),
                "model_variance":
                0.05,
                "model_friction_variance":
                0.05,
                "ros_traj_topic":
                "trajectory",
                "ros_traj_enb":
                True,
                "in_production":
                False,

                # controller parameters
                "k_r": [10., 10., 10., 10., 10., 10., 10.],
                "alpha":
                10.,
                "V_max":
                1e-2,
                "r_norm_threshold":
                1e-7,

                # joint filter
                "enable_joint_filter":
                False,
                "vfilter_a": [0.1, 0.1],
                "vfilter_b": [0.7, 0.2, 0.1],
                # force closure starting point
                "has_init_pos":
                True,
                "init_pos": [
                    2.61799388, -1.57079633, -1.57079633, 1.57079633, 0.,
                    1.57079633, 1.57079633
                ],
            }]),
        Node(package='roahm_planner',
             executable='play_bernstein_traj.py',
             name='traj_player',
             parameters=[{
                 "data_path": LaunchConfiguration("data_path"),
             }])
    ])
