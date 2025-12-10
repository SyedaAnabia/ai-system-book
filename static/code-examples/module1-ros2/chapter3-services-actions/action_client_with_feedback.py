import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

# Using the standard Fibonacci action interface as an example
from action_tutorials_interfaces.action import Fibonacci


class HumanoidActionClient(Node):
    def __init__(self):
        super().__init__('humanoid_action_client')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,  # Using Fibonacci as a placeholder
            'fibonacci_test'  # Using test action name
        )

    def send_goal(self, order):
        """Send a goal to the action server."""
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create the goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self.get_logger().info(f'Sending goal: order={order}')
        
        # Send the goal and register callbacks for feedback and result
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Register callback for when the result is received
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return self._send_goal_future

    def goal_response_callback(self, future):
        """Handle the result of sending the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return
        
        self.get_logger().info('Goal accepted by server, waiting for result...')
        
        # Get the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        self.get_logger().info(
            f'Received feedback: Sequence length = {len(feedback_msg.sequence)}, '
            f'Current sequence: {feedback_msg.sequence}'
        )

    def get_result_callback(self, future):
        """Handle the final result from the action server."""
        result = future.result().result
        self.get_logger().info(f'Result received: {result.sequence}')
        
        # Process the result
        self.get_logger().info(f'Final sequence has {len(result.sequence)} elements')


def main(args=None):
    rclpy.init(args=args)
    
    action_client = HumanoidActionClient()
    
    # Send a goal to generate a Fibonacci sequence of order 5
    future = action_client.send_goal(5)
    
    try:
        # Spin to process callbacks
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Interrupted during action execution')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()