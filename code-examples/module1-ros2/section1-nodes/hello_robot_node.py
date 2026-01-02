import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class HelloRobotNode(Node):
    def __init__(self):
        super().__init__('hello_robot_node')
        
        # Declare parameters with defaults
        self.declare_parameter('robot_name', 'ROSbot')
        self.declare_parameter('status_interval', 2.0)  # seconds
        self.declare_parameter('greeting_message', 'Hello from')
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.interval = self.get_parameter('status_interval').get_parameter_value().double_value
        self.greeting = self.get_parameter('greeting_message').get_parameter_value().string_value
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        
        # Create timer
        self.timer = self.create_timer(self.interval, self.status_callback)
        self.status_count = 0
        
        self.get_logger().info(
            f'Hello Robot Node started for {self.robot_name} '
            f'with status interval {self.interval}s'
        )

    def status_callback(self):
        """Publish robot status message"""
        msg = String()
        timestamp = datetime.now().strftime("%H:%M:%S")
        msg.data = f'{self.greeting} {self.robot_name} - Status #{self.status_count} at {timestamp}'
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.status_count += 1

def main(args=None):
    rclpy.init(args=args)
    
    hello_robot_node = HelloRobotNode()
    
    try:
        rclpy.spin(hello_robot_node)
    except KeyboardInterrupt:
        hello_robot_node.get_logger().info('Hello Robot Node interrupted by user')
    finally:
        hello_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()