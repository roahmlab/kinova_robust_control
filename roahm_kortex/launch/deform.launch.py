#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    ip_launch_arg = DeclareLaunchArgument(
        "ip",
        default_value=TextSubstitution(text="192.168.1.10"),
        description="ip address of the robot")
    model_launch_arg = DeclareLaunchArgument(
        "model_path",
        default_value=TextSubstitution(text="./models/mjcf/kinova3/gen3.mjcf"),
        description="path to robot model")
    debug_launch_arg = DeclareLaunchArgument(
        "debug", default_value="True", description="whether to enable debug")

    has_init_pos_launch_arg = DeclareLaunchArgument(
        "has_init_pos",
        default_value="False",
        description="whether to enable debug")
    return LaunchDescription([
        ip_launch_arg,
        model_launch_arg,
        debug_launch_arg,
        has_init_pos_launch_arg,
        Node(
            package='roahm_kortex',
            executable='robust_control_system',
            name='robust_control_system',
            parameters=[{
                "model_path":
                LaunchConfiguration("model_path"),
                "ip":
                LaunchConfiguration("ip"),
                "model_variance":
                0.03,
                # variances for different parts
                "friction_variance":
                0.05,
                "ee_variance": # end effector
                0.03,
                "ros_traj_enb":
                True,
                "ros_traj_topic":
                "trajectory",
                "debug":
                LaunchConfiguration("debug"),
                "in_production":
                True,

                # disable control
                "k_r": [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                "alpha":
                10.0,
                "V_max":
                3.0e-2,
                "r_norm_threshold":
                1e-9,

                # force closure starting point
                "has_init_pos":
                LaunchConfiguration("has_init_pos"),
                "init_pos": [
                3.9270, -0.5236, 0, -2.0944, 0, 1.0472, 0
                ], 
                # 3.75, -1.0472, 0, -2.0944, 0, 1.5708, 0 # standard hardware
                # 3.9270, -0.5236, 0, -2.0944, 0, 1.0472, 0
                # 3.9270, -1.0472, 0, -2.0944, 0, 1.5708, 0
                # 2.61799388, -1.57079633, -1.57079633, 1.57079633, 0.0, 1.57079633, 1.57079633 # ford demo pose
            }])
    ])
