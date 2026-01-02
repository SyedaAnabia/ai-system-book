import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        # Initialize the node with name 'minimal_node'
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node has started')

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create the node
    minimal_node = MinimalNode()
    
    # Spin to keep the node alive and process callbacks
    rclpy.spin(minimal_node)
    
    # Clean up and shutdown
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()