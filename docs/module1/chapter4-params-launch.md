# ROS 2 Parameters and Launch Files - Configuration and Orchestration

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain the purpose of ROS 2 parameters and how they differ from hard-coded values
2. Implement parameter declarations, access, and validation in ROS 2 nodes
3. Create and manage launch files for orchestrating multi-node robotic systems
4. Use launch file arguments and substitutions for flexible deployments
5. Apply Quality of Service (QoS) profiles appropriately for different communication needs
6. Debug and troubleshoot parameter and launch file configurations
7. Design humanoid robot systems using proper parameter management and orchestration

## Prerequisites

Before starting this chapter, you should have:

- Completed Chapters 1-3 of this book (ROS 2 basics, topics, services/actions)
- Basic Python programming knowledge
- ROS 2 Iron or Humble installed and properly configured
- Comfort with command-line interfaces
- Understanding of robot simulation environments (Gazebo/RViz2)

## 1. Understanding ROS 2 Parameters

ROS 2 parameters provide a way to configure nodes at runtime without recompiling code. They enable dynamic reconfiguration of node behavior and facilitate system adaptation to different environments or operational modes. This is particularly important in humanoid robotics where robots may need to adjust their behavior based on environmental conditions or operational requirements.

### 1.1 What are Parameters?

Parameters in ROS 2 are named values that belong to a specific node. They can be:
- Set at node startup via launch files or parameter files
- Modified at runtime through command-line tools or other nodes
- Bound to specific types and ranges for validation

Parameters differ from topics in that they provide a key-value store rather than a communication channel. They differ from services in that they don't require request-response patterns but instead provide persistent state that can be changed dynamically.

### 1.2 Parameter Types and Constraints

ROS 2 supports several parameter types:
- Integer (int64)
- Double (float64)
- String
- Boolean
- Lists (integer, double, string, boolean)

Parameters can include constraints to ensure valid values:
- Integer/double ranges
- Allowed string values
- Read-only parameters
- Dynamic validation through callbacks

### 1.3 Creating Your First Parameter Node

Let's start with a simple node that declares and uses parameters:

```python
# File: simple_parameter_node.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterNotDeclaredException


class SimpleParameterNode(Node):
    def __init__(self):
        super().__init__('simple_parameter_node')
        
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
                type_=ParameterType.PARAMETER_STRING
            )
        )
        
        self.declare_parameter(
            'max_velocity', 
            1.0,
            ParameterDescriptor(
                description='Maximum velocity for movement',
                type_=ParameterType.PARAMETER_DOUBLE,
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

    def parameter_callback(self, params):
        """Handle parameter changes."""
        for param in params:
            if param.name == 'max_velocity':
                if param.value > 5.0:
                    self.get_logger().warn(f'Velocity {param.value} may be unsafe for humanoids')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    
    simple_param_node = SimpleParameterNode()
    
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

## 2. Working with Parameters in Depth

### 2.1 Parameter Callbacks and Validation

Parameter callbacks allow nodes to validate incoming parameter changes and react to them appropriately. This is essential for humanoid robotics where safety and operational constraints must be maintained.

### 2.2 YAML Configuration Files

Parameters can be loaded from YAML files, which is more convenient for complex systems:

```yaml
# robot_params.yaml
/**:
  ros__parameters:
    robot_name: "humanoid_alpha"
    operational_mode: "cautious"
    max_velocity: 2.0
    joint_limits:
      hip_pitch_min: -0.5
      hip_pitch_max: 1.0
      knee_pitch_min: 0.0
      knee_pitch_max: 2.0
```

### 2.3 Command-Line Parameter Setting

You can also set parameters from the command line:

```bash
# Set a parameter for a running node
ros2 param set /simple_parameter_node robot_name "advanced_robot"

# Get a parameter value
ros2 param get /simple_parameter_node max_velocity

# List all parameters for a node
ros2 param list /simple_parameter_node
```

## 3. Introduction to Launch Files

Launch files provide a way to start multiple nodes at once, configure them, and manage their lifecycle. For humanoid robots, which typically consist of many interconnected nodes, launch files are essential for coordinated startup and configuration.

### 3.1 Why Launch Files Matter

Without launch files, starting a complex robotic system would require manually opening multiple terminals and running commands sequentially. Launch files automate this process, ensuring proper startup order and configuration.

### 3.2 Launch File Formats

ROS 2 supports multiple launch file formats:
- Python (recommended): Most flexible, supports complex logic
- XML: Declarative, but less flexible
- YAML: Declarative, but limited functionality

### 3.3 Basic Launch File Structure

Here's a basic launch file structure:

```python
# basic_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='simple_parameter_node',
            name='robot_controller',
            parameters=[
                {'robot_name': 'launch_robot'},
                {'operational_mode': 'demo'},
                {'max_velocity': 1.5}
            ],
            output='screen'
        )
    ])
```

## 4. Creating Launch Files

### 4.1 Python Launch API Fundamentals

The Python launch API provides the most flexibility for creating complex launch scenarios. It allows for conditional execution, argument passing, and dynamic node creation.

### 4.2 Launching Single and Multiple Nodes

Launch files can start single nodes or complex multi-node systems:

```python
# multi_node_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot1',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Robot controller node
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='controller',
            namespace=robot_namespace,
            parameters=[
                {'use_sim_time': use_sim_time},
                # More parameters here
            ],
            remappings=[
                ('/cmd_vel', 'cmd_vel'),
                ('/odom', 'odom'),
            ]
        ),
        
        # Sensor processor node
        Node(
            package='my_robot_package',
            executable='sensor_processor',
            name='sensors',
            namespace=robot_namespace,
            parameters=[
                {'use_sim_time': use_sim_time},
                # More parameters here
            ]
        )
    ])
```

### 4.3 Node Remapping and Namespacing

Launch files support remapping topics and namespacing nodes, which is crucial for multi-robot systems and avoiding naming conflicts.

## 5. Advanced Launch Techniques

### 5.1 Conditional Launching (if/unless)

Launch files can conditionally start nodes based on arguments or other conditions:

```python
# conditional_launch.py
from launch import LaunchDescription, LaunchCondition
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug_enabled = LaunchConfiguration('debug')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug nodes'
        ),
        
        # Debug node that only runs if debug is enabled
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            condition=IfCondition(debug_enabled)
        )
    ])
```

### 5.2 Launch File Composition Patterns

Complex systems often use composition patterns to include other launch files:

```python
# main_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Include other launch files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('my_robot_description'),
                    'launch',
                    'robot_description.launch.py'
                ])
            )
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('my_robot_control'),
                    'launch',
                    'controllers.launch.py'
                ])
            )
        )
    ])
```

## 6. Quality of Service (QoS) Profiles

QoS profiles control how messages are delivered between nodes. For humanoid robotics applications where safety and real-time performance are critical, choosing appropriate QoS settings is essential.

### 6.1 QoS Profiles Explained

Different QoS profiles provide different guarantees for reliability, durability, and history of messages:

- **Reliability**: Whether messages are guaranteed to be delivered
- **Durability**: Whether late-joining subscribers receive previous messages
- **History**: How many messages to store for delivery

## 7. Practical Example: Humanoid Robot System

Let's put everything together with a practical example of a humanoid robot system:

```python
# humanoid_system_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_config = LaunchConfiguration('robot_config')
    
    # Composable nodes container for efficiency
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node'
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='pointcloud_node'
            )
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='config/default.yaml',
            description='Robot configuration file'
        ),
        
        # Robot state publisher with parameters
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                robot_config,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Joint state broadcaster
        Node(
            package='joint_state_broadcaster',
            executable='joint_state_broadcaster',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Perception container
        perception_container,
        
        # Main controller with parameter validation
        Node(
            package='humanoid_controller',
            executable='main_controller',
            parameters=[
                robot_config,
                {'use_sim_time': use_sim_time}
            ],
            on_exit=RegisterEventHandler(
                OnProcessStart(
                    target_action=perception_container,
                    on_start=lambda event: print("Controller started after perception container")
                )
            )
        )
    ])
```

## 8. Debugging and Tools

### 8.1 ROS 2 CLI Tools for Parameters and Launch

Several command-line tools help with debugging parameters and launches:

```bash
# List all parameters for a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name param_name

# Set parameter value
ros2 param set /node_name param_name new_value

# List all available launch files in a package
ros2 launch <package_name>

# Run a launch file
ros2 launch <package_name> <launch_file.py>
```

### 8.2 Visualization and Monitoring

Tools like `rqt_reconfigure` allow real-time parameter adjustment, while `rqt_graph` visualizes the node graph structure of your system.

## Exercises

### Exercise 1: Basic Parameter Configuration (Beginner)
Create a simple node that accepts a robot configuration parameter and adjusts its behavior accordingly. Test changing the parameter at runtime.

### Exercise 2: Multi-Node Launch System (Intermediate)
Design a launch file that starts multiple nodes representing a basic humanoid robot (e.g., controller, sensors, and visualization) with proper parameter configuration and namespacing.

### Exercise 3: Advanced Parameter Validation (Advanced)
Create a node with complex parameter validation that prevents unsafe configuration changes, particularly for humanoid joint limits and velocity constraints.

## Summary and Next Steps

This chapter covered the essential ROS 2 concepts of parameters and launch files that enable configuration management and system orchestration. You learned how to create nodes with dynamic parameters, organize system startup with launch files, and apply these concepts to humanoid robotics systems.

In the next chapter, we'll explore advanced ROS 2 concepts including composition, lifecycle nodes, and diagnostics for building production-ready robotic systems.

## Additional Resources

1. [ROS 2 Parameters Documentation](https://docs.ros.org/en/rolling/How-To-Guides/Using-Parameters-In-A-Class-Python.html)
2. [Launch System Documentation](https://docs.ros.org/en/rolling/Tutorials/Launch-system.html)
3. [QoS Implementation in RMW](https://github.com/ros2/design/blob/master/articles/qos_changes_and_defaults.md)
4. [Parameter Best Practices](https://index.ros.org/doc/ros2/Releases/Release-Galactic-Geochelone/#parameter-best-practice-guide)
5. [Launch File Best Practices](https://index.ros.org/doc/ros2/Tutorials/Composition/#composition-launch-file)

## FAQ

1. **Q: When should I use parameters vs topics vs services?**  
   A: Use parameters for configuration values that change infrequently and need to persist across node restarts. Use topics for continuous data streaming, and services for request-response interactions.

2. **Q: How do I share parameters between nodes?**  
   A: Parameters are node-specific by default. To share configuration values, you can use a parameter server node, share parameter files, or use services to query configuration.

3. **Q: What's the difference between launch files and bringup scripts?**  
   A: Launch files are the modern ROS 2 approach using the launch system, while bringup scripts are older shell scripts that manually start nodes. Launch files provide better error handling, parameter configuration, and lifecycle management.