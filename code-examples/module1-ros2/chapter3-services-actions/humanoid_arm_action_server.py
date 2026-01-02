import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

# Using the standard Fibonacci action interface as an example
from action_tutorials_interfaces.action import Fibonacci


class HumanoidWalkActionServer(Node):
    def __init__(self):
        super().__init__('humanoid_walk_action_server')
        
        # Create action server with callback group to handle multiple goals
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Using Fibonacci as a placeholder since we don't have humanoid_locomotion_interfaces
            'fibonacci_test',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.get_logger().info('Demo action server is ready.')

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info(
            f'Received goal request: order={goal_request.order}'
        )
        
        # Accept the goal
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancellation request."""
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal and return the result."""
        self.get_logger().info('Executing goal...')
        
        # Get the goal request
        goal = goal_handle.request
        order = goal.order
        
        # Initialize feedback and result
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()
        
        # Initialize sequence
        feedback_msg.sequence = [0, 1]
        
        # Main execution loop
        for i in range(1, order):
            # Check if the goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.sequence = feedback_msg.sequence
                self.get_logger().info('Goal canceled')
                return result_msg

            # Calculate next fibonacci number
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate processing time
            time.sleep(1)

        # Goal completed successfully
        if not goal_handle.is_cancel_requested:
            goal_handle.succeed()
            result_msg.sequence = feedback_msg.sequence
            self.get_logger().info(f'Result: {result_msg.sequence}')
        
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    
    demo_server = HumanoidWalkActionServer()
    
    try:
        # Use a multi-threaded executor to handle multiple goals if needed
        executor = MultiThreadedExecutor(num_threads=4)
        rclpy.spin(demo_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        demo_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()