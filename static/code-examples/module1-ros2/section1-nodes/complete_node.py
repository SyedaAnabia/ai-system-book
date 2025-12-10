import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String

class CompleteNode(Node):
    def __init__(self):
        # Initialize the node with name 'complete_node'
        super().__init__('complete_node')
        
        # Different logging levels for different purposes
        self.get_logger().debug('Debug: Complete node initialization started')
        self.get_logger().info('Info: Complete node initialized successfully')
        
        # Create a publisher
        self.publisher = self.create_publisher(String, 'node_status', qos_profile_system_default)
        
        # Create a timer that calls the callback function every 1.5 seconds
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # Parameter declaration
        self.declare_parameter('robot_name', 'default_robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.get_logger().info(f'Robot name parameter set to: {self.robot_name}')

    def timer_callback(self):
        """Callback function called by the timer"""
        msg = String()
        msg.data = f'Robot {self.robot_name} status update: cycle {self.i}'
        
        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        
        self.i += 1
        
        # Example of conditional logging
        if self.i % 10 == 0:
            self.get_logger().warn(f'Warning: Node has run {self.i} cycles')
        
        # Example of error handling
        if self.i > 100:
            self.get_logger().error('Error: Cycle count exceeded expected value')
            self.timer.cancel()  # Stop the timer

def main(args=None):
    """Main function to run the node"""
    rclpy.init(args=args)
    
    complete_node = CompleteNode()
    
    try:
        # Keep the node running
        rclpy.spin(complete_node)
    except KeyboardInterrupt:
        # Handle graceful shutdown when Ctrl+C is pressed
        complete_node.get_logger().info('Node interrupted by user, shutting down')
    except Exception as e:
        # Log any exceptions
        complete_node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Ensure proper cleanup
        complete_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()