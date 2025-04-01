#!/usr/bin/env python3

import rclpy
from roahm_msgs.srv import GripperAction
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__("cartesian_test")
        self.reset_gripper_cli = self.create_client(GripperAction, "ros_kortex_system/reset_gripper")
        self.close_gripper_cli = self.create_client(GripperAction, "ros_kortex_system/close_gripper")

        while not self.reset_gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("waiting for service")
        while not self.close_gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("waiting for service")
    
    def reset_gripper(self):
        self.future = self.reset_gripper_cli.call_async(GripperAction.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def close_gripper(self):
        self.future = self.close_gripper_cli.call_async(GripperAction.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

if __name__ == '__main__':
    rclpy.init()

    test_node = TestNode()
    response = test_node.reset_gripper()
    response = test_node.close_gripper()
    response = test_node.reset_gripper()
    test_node.get_logger().info("success? " + str(response))

    test_node.destroy_node()
    rclpy.shutdown()