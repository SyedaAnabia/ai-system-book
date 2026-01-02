#!/usr/bin/env python3

"""
Comparison Example: Topic vs Service vs Action

This example demonstrates the differences between the three main communication patterns in ROS 2:
- Topics: Asynchronous, continuous data streaming
- Services: Synchronous, request-response pattern
- Actions: Goal-oriented with feedback and result
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_tutorials_interfaces.action import Fibonacci
import time
import rclpy.executors


class TopicExample(Node):
    def __init__(self):
        super().__init__('topic_example')
        
        # Publisher for continuous data
        self.publisher = self.create_publisher(String, 'sensor_data_topic', 10)
        
        # Subscriber to process messages
        self.subscription = self.create_subscription(
            String,
            'sensor_data_topic',
            self.sensor_callback,
            10
        )
        
        # Timer to periodically publish topic data
        self.topic_timer = self.create_timer(1.0, self.publish_sensor_data)
        self.counter = 0
        self.get_logger().info("Topics initialized")

    def publish_sensor_data(self):
        """Publish sensor data to topic (simulating continuous sensor readings)"""
        msg = String()
        msg.data = f"Sensor reading #{self.counter}: value={self.counter * 10}"
        self.publisher.publish(msg)
        self.get_logger().info(f"[TOPIC] Published: {msg.data}")
        self.counter += 1

    def sensor_callback(self, msg):
        """Callback for processing topic data"""
        self.get_logger().info(f"[TOPIC] Received: {msg.data}")


class ServiceExample(Node):
    def __init__(self):
        super().__init__('service_example')
        
        # Service server for request-response
        self.service = self.create_service(AddTwoInts, 'calculation_service', self.add_callback)
        self.get_logger().info("Service initialized")

    def add_callback(self, request, response):
        """Service callback for request-response"""
        self.get_logger().info(f"[SERVICE] Received request: {request.a} + {request.b}")
        response.sum = request.a + request.b
        self.get_logger().info(f"[SERVICE] Responding with: {response.sum}")
        return response


class ActionExample(Node):
    def __init__(self):
        super().__init__('action_example')
        
        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("Action initialized")

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info('[ACTION] Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancellation request."""
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal and return the result."""
        self.get_logger().info('Executing goal...')

        # Get the goal request
        order = goal_handle.request.order

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
            
            self.get_logger().info(f'[ACTION] Feedback: {feedback_msg.sequence}')
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate processing time
            time.sleep(1)

        # Goal completed successfully
        if not goal_handle.is_cancel_requested:
            goal_handle.succeed()
            result_msg.sequence = feedback_msg.sequence
            self.get_logger().info(f'[ACTION] Result: {result_msg.sequence}')
        
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    
    # Create nodes for each communication pattern
    topic_node = TopicExample()
    service_node = ServiceExample()
    action_node = ActionExample()
    
    # Use a multi-threaded executor to run all nodes together
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(topic_node)
    executor.add_node(service_node)
    executor.add_node(action_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down comparison example")
    finally:
        topic_node.destroy_node()
        service_node.destroy_node()
        action_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()