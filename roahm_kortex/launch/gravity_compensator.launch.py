#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

namespace_name: str = "gravity_compensator"


def generate_launch_description():
    ip_launch_arg = DeclareLaunchArgument(
        "ip", default_value=TextSubstitution(text="192.168.1.11"))
    model_launch_arg = DeclareLaunchArgument(
        "model_path",
        default_value=TextSubstitution(
            text="./models/mjcf/kinova3/gen3_nofa.mjcf"),
        description="path to robot model")
    gripper_launch_arg = DeclareLaunchArgument(
        "gripper_path",
        default_value=TextSubstitution(
            text="./models/mjcf/robotiq_85_new.mjcf"))
    is_gripper_open_arg = DeclareLaunchArgument(
        "is_gripper_open",
        default_value='True',
        description="whether to open gripper")
    return LaunchDescription([
        ip_launch_arg,
        model_launch_arg,
        is_gripper_open_arg,
        gripper_launch_arg,
        Node(
            namespace=namespace_name,
            package='roahm_kortex',
            executable='robust_control_system',
            name='robust_control_system',
            parameters=[{
                "model_path":
                LaunchConfiguration("model_path"),
                "gripper_path":
                LaunchConfiguration("gripper_path"),
                "ip":
                LaunchConfiguration("ip"),
                "ros_traj_enb":
                False,

                # disable control
                "alpha":
                0.,
                "r_norm_threshold":
                1e-7,
                "is_gripper_open":
                LaunchConfiguration("is_gripper_open"),

                # force closure starting point
                "has_init_pos":
                False,
            }])
    ])
