#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kill_switch.action import ConnectKillSwitch

class KillSwitchNode(Node):
    def _init_(self):
        super()._init_('kill_switch_node')

        self._client = ActionClient(self, ConnectKillSwitch, 'connect_kill_switch')
        self.get_logger().info("Kill Switch Node started. Waiting to connect to race car...")

        self._client.wait_for_server()
        self.send_goal()

    def send_goal(self):
        goal_msg = ConnectKillSwitch.Goal()
        goal_msg.connect = True

        self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Kill Switch connection was rejected by the race car.")
            return
        self.get_logger().info("Kill Switch connection accepted.")

        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Race Car Feedback: {feedback.status}")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Final result: acknowledged = {result.acknowledged}")

def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchNode()
    rclpy.spin(node)
    rclpy.shutdown()

if _name_ == '_main_':
    main()