#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from kill_switch.action import ConnectKillSwitch

class KillSwitchServer(Node):
    def __init__(self):
        super().__init__('kill_switch_server')

        self._action_server = ActionServer(
            self,
            ConnectKillSwitch,
            'connect_kill_switch',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Kill Switch Action Server is active.")

    def goal_callback(self, goal_request):
        self.get_logger().info("Kill Switch wants to connect.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Kill Switch disconnected! Initiating emergency stop.")
        # TODO: We'll add /drive stop publishing here next
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        feedback = ConnectKillSwitch.Feedback()
        feedback.status = "Connected to Kill Switch. Driving enabled."
        goal_handle.publish_feedback(feedback)

        result = ConnectKillSwitch.Result()
        result.acknowledged = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()