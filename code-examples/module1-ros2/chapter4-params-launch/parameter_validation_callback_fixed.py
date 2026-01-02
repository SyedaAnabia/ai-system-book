# parameter_validation_callback.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange
import numpy as np


class ParameterValidationNode(Node):
    def __init__(self):
        super().__init__('parameter_validation_node')

        # Define parameter constraints and defaults
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.msg import FloatingPointRange

        # Create parameter descriptor for walking step size
        param_desc1 = ParameterDescriptor()
        param_desc1.description = 'Size of each walking step in meters for humanoid robot'
        param_desc1.additional_constraints = 'Must be between 0.1 and 0.5 meters for safe operation'
        float_range1 = FloatingPointRange()
        float_range1.from_value = 0.1
        float_range1.to_value = 0.5
        float_range1.step = 0.01
        param_desc1.floating_point_range = [float_range1]

        # Create parameter descriptor for max joint velocity
        param_desc2 = ParameterDescriptor()
        param_desc2.description = 'Maximum allowable joint velocity in radians per second'
        param_desc2.additional_constraints = 'Must not exceed 3.0 rad/s to prevent damage to humanoid actuators'
        float_range2 = FloatingPointRange()
        float_range2.from_value = 0.1
        float_range2.to_value = 3.0
        float_range2.step = 0.01
        param_desc2.floating_point_range = [float_range2]

        # Create parameter descriptor for operational mode
        param_desc3 = ParameterDescriptor()
        param_desc3.description = 'Current operational mode of the robot'
        param_desc3.additional_constraints = 'Valid values are: "normal", "cautious", "maintenance", "disabled"'

        # Declare parameters with proper descriptors
        self.declare_parameter(
            'walking_step_size_m',
            0.3,
            param_desc1
        )

        self.declare_parameter(
            'max_joint_velocity_rad_per_sec',
            1.0,
            param_desc2
        )

        self.declare_parameter(
            'robot_operational_mode',
            'normal',
            param_desc3
        )

        # Set up parameter change callback
        self.add_on_set_parameters_callback(self.validate_parameters)

        # Get initial parameter values
        self.step_size = self.get_parameter('walking_step_size_m').value
        self.max_velocity = self.get_parameter('max_joint_velocity_rad_per_sec').value
        self.op_mode = self.get_parameter('robot_operational_mode').value

        self.get_logger().info(f'Initial parameters set: step_size={self.step_size}, max_vel={self.max_velocity}, op_mode={self.op_mode}')

    def validate_parameters(self, parameter_list):
        """
        Validate parameter changes before accepting them.

        Args:
            parameter_list: List of Parameter objects to validate

        Returns:
            SetParametersResult indicating success or failure
        """
        result = SetParametersResult()
        result.successful = True
        result.reason = 'All parameters validated successfully'

        for param in parameter_list:
            if param.name == 'walking_step_size_m':
                if param.type != Parameter.Type.PARAMETER_DOUBLE:
                    result.successful = False
                    result.reason = f'walking_step_size_m must be a double, got {param.type}'
                    return result

                if param.value < 0.1 or param.value > 0.5:
                    result.successful = False
                    result.reason = f'walking_step_size_m {param.value} is outside safe range [0.1, 0.5]'
                    return result

                self.get_logger().info(f'Approved step size change to {param.value}m')

            elif param.name == 'max_joint_velocity_rad_per_sec':
                if param.type != Parameter.Type.PARAMETER_DOUBLE:
                    result.successful = False
                    result.reason = f'max_joint_velocity_rad_per_sec must be a double, got {param.type}'
                    return result

                if param.value < 0.1 or param.value > 3.0:
                    result.successful = False
                    result.reason = f'max_joint_velocity_rad_per_sec {param.value} is outside safe range [0.1, 3.0]'
                    return result

                if param.value > 1.5:
                    self.get_logger().warn(f'High joint velocity {param.value} rad/s may stress actuators')

                self.get_logger().info(f'Approved max velocity change to {param.value} rad/s')

            elif param.name == 'robot_operational_mode':
                if param.type != Parameter.Type.PARAMETER_STRING:
                    result.successful = False
                    result.reason = f'robot_operational_mode must be a string, got {param.type}'
                    return result

                valid_modes = ['normal', 'cautious', 'maintenance', 'disabled']
                if param.value not in valid_modes:
                    result.successful = False
                    result.reason = f'robot_operational_mode {param.value} is not in valid list: {valid_modes}'
                    return result

                if param.value == 'disabled':
                    self.get_logger().warn('Robot operational mode set to DISABLED - all motion commands will be rejected')
                elif param.value == 'cautious':
                    self.get_logger().info('Robot operational mode set to CAUTIOUS - reduced speeds and conservative behaviors enabled')

                self.get_logger().info(f'Approved operational mode change to {param.value}')

        return result


def main(args=None):
    rclpy.init(args=args)

    param_validation_node = ParameterValidationNode()

    try:
        # Keep the node running to handle parameter changes
        rclpy.spin(param_validation_node)
    except KeyboardInterrupt:
        param_validation_node.get_logger().info('Parameter validation node shutting down')
    finally:
        param_validation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()