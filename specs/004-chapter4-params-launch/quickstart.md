# Quickstart Guide: Chapter 4 - ROS 2 Parameters and Launch Files

**Feature**: 004-chapter4-params-launch
**Date**: 2025-12-09

This quickstart guide provides the essential prerequisites and setup steps needed to work with the examples in Chapter 4: "ROS 2 Parameters and Launch Files - Configuration and Orchestration" of the AI Systems in the Physical World book.

## Prerequisites

Before starting with Chapter 4, ensure you have completed Chapters 1-3 and have the following:

### System Requirements
- Operating System: Ubuntu 22.04 LTS, Windows 10/11 with WSL2, or macOS 12+
- RAM: 8GB minimum, 16GB recommended
- Storage: 5GB free space for examples and temporary files
- Python 3.8+ with pip package manager

### Software Requirements
- ROS 2 Iron/Humble installed and properly sourced
- Python 3.8+ with pip
- Git version control system
- Visual Studio Code (or similar IDE) with Python extensions
- YAML editor for configuration files

### Chapter Prerequisites
- Understanding of basic ROS 2 concepts from Chapter 1 (nodes, basic architecture)
- Understanding of topic communication from Chapter 2 (publishers/subscribers)
- Understanding of service/action communication from Chapter 3
- Ability to run basic ROS 2 commands (ros2 topic, ros2 service, etc.)
- Basic Python programming knowledge

## Setting Up for Chapter 4

### 1. Verify Your ROS 2 Installation
```bash
# Check ROS 2 version
ros2 --version

# Verify ROS 2 is sourced
echo $ROS_DISTRO
# Should output "iron"

# List available commands
ros2
```

### 2. Create a Workspace for Chapter 4 Examples
```bash
# Create a workspace for examples
mkdir -p ~/chapter4_ws/src
cd ~/chapter4_ws

# Source ROS 2 environment
source /opt/ros/iron/setup.bash  # or humble
```

### 3. Install Chapter-Specific Dependencies
```bash
# Install additional packages needed for parameters and launch examples
sudo apt update
sudo apt install python3-yaml ros-iron-launch-* ros-iron-launch-ros

# Verify launch libraries are available
python3 -c "import launch; import launch_ros; print('Launch libraries available')"

# Verify YAML library
python3 -c "import yaml; print('PyYAML library available')"
```

## Running Parameter Examples

### 1. Navigate to Chapter Examples
```bash
# The code examples are organized by section
cd ~/chapter4_ws/src/
ls -la
```

### 2. Example: Creating a Parameter Server
First, create a basic parameter server package:

```bash
cd ~/chapter4_ws/src
ros2 pkg create --build-type ament_python parameter_server_examples
cd parameter_server_examples/parameter_server_examples
```

Create `simple_parameter_server.py`:
```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterDescriptor
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange


class SimpleParameterServer(Node):
    def __init__(self):
        super().__init__('simple_parameter_server')
        
        # Create parameter descriptor for walking step size
        param_desc = ParameterDescriptor()
        param_desc.description = 'Size of each walking step in meters for humanoid robot'
        param_desc.additional_constraints = 'Must be between 0.1 and 0.5 meters for safe operation'
        float_range = FloatingPointRange()
        float_range.from_value = 0.1
        float_range.to_value = 0.5
        float_range.step = 0.01
        param_desc.floating_point_range = [float_range]

        # Declare parameters with default values and descriptions
        self.declare_parameter(
            'walking_step_size_m',
            0.3,
            param_desc
        )
        
        self.declare_parameter(
            'operational_mode', 
            'normal',
            ParameterDescriptor(
                description='Operating mode of the robot',
                additional_constraints='Valid values: "normal", "cautious", "maintenance", "disabled"'
            )
        )
        
        self.declare_parameter(
            'max_velocity', 
            1.0,
            ParameterDescriptor(
                description='Maximum velocity for movement',
                additional_constraints='Must not exceed 3.0 m/s',
                floating_point_range=[FloatingPointRange(from_value=0.1, to_value=3.0, step=0.01)]
            )
        )

        # Get parameter values
        step_size = self.get_parameter('walking_step_size_m').value
        operational_mode = self.get_parameter('operational_mode').value
        max_velocity = self.get_parameter('max_velocity').value

        self.get_logger().info(f'Step size: {step_size}')
        self.get_logger().info(f'Operational mode: {operational_mode}')
        self.get_logger().info(f'Max velocity: {max_velocity}')

        # Create a parameter callback to handle dynamic changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, parameter_list):
        """Handle parameter changes."""
        from rcl_interfaces.msg import SetParametersResult

        for param in parameter_list:
            if param.name == 'walking_step_size_m' and param.value > 0.45:
                self.get_logger().warn(f'Large step size {param.value}m may be unsafe for humanoid robots')
        
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
```

### 3. Build and Run the Parameter Example
```bash
# From the workspace root
cd ~/chapter4_ws

# Build the package
colcon build --packages-select parameter_server_examples

# Source the workspace
source install/setup.bash

# Run the parameter node with overrides
ros2 run parameter_server_examples simple_parameter_server --ros-args \
  -p walking_step_size_m:=0.25 \
  -p operational_mode:=cautious \
  -p max_velocity:=0.8
```

## Running Launch Examples

### 1. Create Launch File Example
In your package directory, create a launch directory:
```bash
mkdir -p ~/chapter4_ws/src/parameter_server_examples/launch
```

Create `simple_launch.py` in the launch directory:
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parameter_server_examples',
            executable='simple_parameter_server',
            name='param_demo_node',
            parameters=[
                {'walking_step_size_m': 0.2},
                {'operational_mode': 'demo'},
                {'max_velocity': 1.2}
            ],
            output='screen'
        )
    ])
```

### 2. Run the Launch File
```bash
# Make sure you're in the workspace root and sourced
cd ~/chapter4_ws
source install/setup.bash

# Run the launch file
ros2 launch parameter_server_examples simple_launch.py
```

## Essential ROS 2 Commands for Chapter 4

### Parameter Commands
```bash
# List all parameters for a node
ros2 param list /param_demo_node

# Get the value of a specific parameter
ros2 param get /param_demo_node walking_step_size_m

# Set the value of a parameter
ros2 param set /param_demo_node operational_mode maintenance

# Describe parameters for a node
ros2 param describe /param_demo_node walking_step_size_m

# Load parameters from a YAML file
ros2 param load /param_demo_node /path/to/params.yaml

# Dump all parameters to a YAML file
ros2 param dump /param_demo_node
```

### Launch Commands
```bash
# Run a launch file
ros2 launch package_name launch_file.py

# Run a launch file with arguments
ros2 launch package_name launch_file.py arg_name:=value

# Dry-run a launch file to see what will be launched
ros2 launch --dry-run package_name launch_file.py

# Get information about launch files in a package
ros2 launch package_name
```

## Testing Your Setup

### 1. Verify Parameter Declaration
```bash
# Terminal 1: Run a node with parameters
source ~/chapter4_ws/install/setup.bash
ros2 run parameter_server_examples simple_parameter_server

# Terminal 2: Check parameters
source ~/chapter4_ws/install/setup.bash
ros2 param list
# Should show parameters from your node
```

### 2. Verify Launch System
```bash
# Create and run a simple launch file
cd ~/chapter4_ws/src/
mkdir simple_launch_demo
mkdir simple_launch_demo/launch
touch simple_launch_demo/setup.py

# Create a basic launch file
cat <<EOF > simple_launch_demo/launch/hello_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker_node',
            parameters=[{'message': 'Hello from launch!'}],
            output='screen'
        )
    ])
EOF

# Build and run
cd ~/chapter4_ws
colcon build --packages-select simple_launch_demo
source install/setup.bash
ros2 launch simple_launch_demo hello_launch.py
```

## Troubleshooting Common Issues

### Common Issue 1: Parameter Declaration After Node Creation
If you get "parameter has already been declared" errors:
- Ensure all parameters are declared in the constructor before any other operations
- Use the declare_parameter method before getting parameter values

### Common Issue 2: Launch File Not Found
If launch files don't run properly:
- Check that the launch file is properly formatted Python code
- Ensure the launch file has executable permissions
- Verify that nodes are properly namespaced in launch files

### Common Issue 3: Parameter Namespaces
If parameters aren't being set correctly:
- Check the full parameter name including namespace
- Use `ros2 param list` to see the actual parameter names
- Verify that nodes are properly namespaced in launch files

### Common Issue 4: "ModuleNotFoundError: No module named 'launch'"
- This means the launch libraries aren't installed
- Install with: `sudo apt install ros-iron-launch-*` (or equivalent for your distribution)

## Next Steps

Once your environment is set up, you're ready to begin with Chapter 4: "ROS 2 Parameters and Launch Files". Start with understanding the parameter system, then move to launch file development, and continue with the practical examples showing how to combine them in humanoid robot applications.

Remember to test each code example as you encounter it in the chapter, and don't hesitate to experiment with the examples to deepen your understanding of ROS 2 configuration management and system orchestration.