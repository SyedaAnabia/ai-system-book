# File: simple_parameter_server.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import FloatingPointRange, IntegerRange


class SimpleParameterServer(Node):
    def __init__(self):
        super().__init__('simple_parameter_server')

        # Declare parameters with default values and descriptions
        self.declare_parameter(
            'robot_name',
            'default_robot',
            ParameterDescriptor(description='Name of the robot')
        )

        self.declare_parameter(
            'operational_mode',
            'normal',
            ParameterDescriptor(
                description='Operating mode of the robot',
                type=ParameterType.PARAMETER_STRING
            )
        )

        self.declare_parameter(
            'max_velocity',
            1.0,
            ParameterDescriptor(
                description='Maximum velocity for movement',
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=10.0, step=0.1)]
            )
        )

        # Get parameter values
        robot_name = self.get_parameter('robot_name').value
        operational_mode = self.get_parameter('operational_mode').value
        max_velocity = self.get_parameter('max_velocity').value

        self.get_logger().info(f'Robot name: {robot_name}')
        self.get_logger().info(f'Operational mode: {operational_mode}')
        self.get_logger().info(f'Max velocity: {max_velocity}')

        # Create a parameter callback to handle dynamic changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, parameter_list):
        """Handle parameter changes."""
        from rcl_interfaces.msg import SetParametersResult

        for param in parameter_list:
            if param.name == 'max_velocity' and param.value > 5.0:
                self.get_logger().warn(f'Velocity {param.value} may be unsafe for humanoid robots')
            elif param.name == 'max_velocity':  # Fixed parameter name to match declaration
                self.get_logger().info(f'Max velocity updated to {param.value}')

        result = SetParametersResult()
        result.successful = True
        return result


def main(args=None):
    rclpy.init(args=args)

    simple_param_node = SimpleParameterServer()

    try:
        rclpy.spin(simple_param_node)
    except KeyboardInterrupt:
        pass
    finally:
        simple_param_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()