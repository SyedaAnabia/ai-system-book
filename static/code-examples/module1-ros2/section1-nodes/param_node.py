import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('log_level', 'INFO')
        self.declare_parameter('safety_enabled', True)
        
        # Access parameters after declaration
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.log_level = self.get_parameter('log_level').get_parameter_value().string_value
        self.safety_enabled = self.get_parameter('safety_enabled').get_parameter_value().bool_value
        
        # Create callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Create publisher for status
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        
        # Create timer based on parameter
        self.timer = self.create_timer(
            1.0 / self.update_rate,  # Interval based on parameter
            self.timer_callback
        )
        
        self.get_logger().info(
            f'Parameter node initialized with robot name: {self.robot_name}, '
            f'update rate: {self.update_rate} Hz, '
            f'safety enabled: {self.safety_enabled}'
        )

    def parameters_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(f'Robot name changed to: {self.robot_name}')
            elif param.name == 'update_rate':
                self.update_rate = param.value
                self.get_logger().info(f'Update rate changed to: {self.update_rate} Hz')
                # Update timer interval
                self.timer.timer_period_ns = int(1.0 / self.update_rate * 1e9)
            elif param.name == 'safety_enabled':
                self.safety_enabled = param.value
                self.get_logger().info(f'Safety state changed to: {self.safety_enabled}')
        
        successful = True  # In a real implementation, you might have validation logic
        return SetParametersResult(successful=successful)

    def timer_callback(self):
        """Timer callback that publishes status based on parameters"""
        msg = String()
        status = f'Robot {self.robot_name} is running'
        if self.safety_enabled:
            status += ' (safety active)'
        else:
            status += ' (safety disabled)'
        
        msg.data = status
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    parameter_node = ParameterNode()
    
    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        parameter_node.get_logger().info('Node interrupted by user')
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()