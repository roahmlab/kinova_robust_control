#!/usr/bin/env python3
from roahm_msgs.srv import GotoName, Goto, Bool, GripperAction
import rclpy
from rclpy.node import Node


class KortexSrvs(Node):
    """
    TODO: consider making this a singleton
    """
    def __init__(self):
        super().__init__('kortex_client')
        self.goto_var = self.create_client(Goto, 'ros_kortex_system/goto')
        self.goto_name_var = self.create_client(GotoName, 'ros_kortex_system/goto_name')
        self.close_gripper_var = self.create_client(GripperAction, 'ros_kortex_system/close_gripper')
        self.reset_gripper_var = self.create_client(GripperAction, 'ros_kortex_system/reset_gripper')

        self.start_srv = self.create_client(Bool, 'ros_kortex_system/run')
        while not self.goto_var.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def goto_name(self, name: str):
        
        req = GotoName.Request()
        req.name = name
        future = self.goto_name_var.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def goto(self, pos):
        req = Goto.Request()
        req.pos = pos
        future = self.goto_var.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def close_gripper(self):
        req = GripperAction.Request()
        future = self.close_gripper_var.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def reset_gripper(self):
        req = GripperAction.Request()
        future = self.reset_gripper_var.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def start_torque_control(self):
        req = Bool.Request()
        self.start_srv.call_async(req)
