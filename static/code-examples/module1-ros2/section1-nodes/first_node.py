import rclpy
from rclpy.node import Node

class FirstNode(Node):
    def __init__(self):
        # Initialize the node with name 'first_node'
        super().__init__('first_node')
        
        # Log a message indicating the node has started
        self.get_logger().info('First node has been started successfully!')
        
        # Create a timer for periodic tasks
        self.timer_period = 2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # This function is called every timer_period seconds
        self.get_logger().info(f'Hello from first node! Count: {self.i}')
        self.i += 1

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create the node
    first_node = FirstNode()
    
    try:
        # Keep the node alive and process callbacks
        rclpy.spin(first_node)
    except KeyboardInterrupt:
        first_node.get_logger().info('Node interrupted by user')
    finally:
        # Clean up and shutdown
        first_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()