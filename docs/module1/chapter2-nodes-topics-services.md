# Understanding ROS 2 Nodes

## 1.1 What is a Node? (300-400 words)

A **node** is the fundamental building block of any ROS 2 system. Think of nodes as individual workers in a factory, each responsible for a specific task. In the ROS 2 ecosystem, nodes are processes that perform computation, communicate with other nodes, and manage robotic functionality.

Unlike a monolithic system where all functionality exists in a single program, ROS 2 uses a distributed architecture. Each node typically handles a specific aspect of robotic functionality - one node might handle sensor data, another might process that data, and a third might control the robot's actuators. This modular approach allows for better organization, easier debugging, and more flexible system design.

The responsibilities of a node in ROS 2 include:

1. **Communication**: Nodes communicate with each other using topics, services, and actions (which we'll explore in later sections).
2. **Data Processing**: Each node typically processes specific data types or performs specific robotic tasks.
3. **Resource Management**: Nodes manage their own resources and lifecycle within the ROS 2 system.
4. **Service Provision**: Nodes can offer services to other nodes or subscribe to services provided by others.

When deciding whether to create a new node or extend an existing one, consider the Single Responsibility Principle. If the new functionality is distinct from the current node's purpose, create a new node. If it's a closely related function, consider extending the existing node or creating a nodelet/component (which we'll cover in advanced concepts).

This modular design enables several advantages:

- **Scalability**: New functionality can be added by simply adding new nodes
- **Maintainability**: Issues in one node don't necessarily affect others
- **Reusability**: Well-designed nodes can be reused across different robotic systems
- **Testability**: Individual nodes can be tested in isolation

ROS 2's distributed nature means nodes can run on the same machine or different machines in a network, allowing for sophisticated robotic architectures that can span multiple physical systems.

## 1.2 Node Anatomy (400-500 words)

Understanding the structure of a ROS 2 node is crucial for effective development. A typical node follows a consistent pattern that allows it to function within the ROS 2 ecosystem.

### Node Initialization
The node initialization process starts with `rclpy.init()`, which initializes the ROS 2 client library for Python. This function must be called before creating any nodes, and it handles command-line arguments and initializes the underlying communication system.

```python
import rclpy
from rclpy.node import Node
```

### Node Class Structure
All ROS 2 nodes inherit from the `Node` class, which provides access to ROS graph functionality, logging, parameter handling, and communication interfaces. The basic structure looks like:

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node-specific initialization
```

The constructor takes a node name as a parameter, which must be unique in the ROS graph. The `super().__init__()` call handles the base node initialization.

### Namespaces and Naming
Nodes can exist within namespaces, which help organize large systems. A node's fully qualified name follows the pattern `/namespace/node_name`. Namespaces allow for multiple instances of the same node type with different configurations.

### Parameters
Nodes can accept parameters that can be set at runtime or during launch. Parameters provide a way to configure node behavior without recompiling. Node parameters are typically declared in the constructor:

```python
self.declare_parameter('param_name', default_value)
```

### Cleanup and Shutdown
Proper cleanup is essential for robust systems. Nodes should handle cleanup operations when shutting down, such as closing file handles, stopping timers, and destroying publishers/subscribers. The ROS 2 system can signal a node to shut down gracefully, and well-designed nodes respond appropriately.

### Code Example 1: Minimal Node

```python
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
```

This minimal example demonstrates the essential components of a ROS 2 node: initialization, a node class inheriting from Node, and proper cleanup procedures.

## 1.3 Creating Your First Node (600-800 words)

Now that we understand the anatomy of a ROS 2 node, let's create your first complete node. This practical exercise will solidify your understanding and give you hands-on experience with the ROS 2 development workflow.

### Prerequisites
Before creating your node, ensure you have:

1. A properly installed ROS 2 environment (Iron or similar)
2. A ROS 2 workspace set up (typically `~/ros2_ws` or similar)
3. Basic Python knowledge
4. Access to a terminal/command prompt

### Step 1: Create Package Structure
In your ROS 2 workspace, create a new package for your node:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_ros2_pkg
```

This command creates a new package with the necessary directory structure for a Python-based ROS 2 package.

Navigate to the package directory:

```bash
cd my_first_ros2_pkg/my_first_ros2_pkg
```

### Step 2: Write the Node Code
Create a new Python file called `first_node.py`:

```python
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
```

This example shows a node that logs a message every 2 seconds, demonstrating the basic structure with a timer callback.

### Step 3: Build the Package
Navigate back to your workspace root and build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_ros2_pkg
```

### Step 4: Run the Node
Source your workspace and run the node:

```bash
source install/setup.bash
ros2 run my_first_ros2_pkg first_node
```

### Step 5: Verify It's Running
In another terminal, verify your node is running:

```bash
# List all active nodes
ros2 node list

# Get information about your specific node
ros2 node info /first_node
```

You should see your node listed with the name `/first_node`.

### Code Example 2: Complete Node with Logging

```python
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
```

This example demonstrates advanced features including publishing, parameters, different log levels, and proper error handling.

### Common Errors and Solutions

**Error 1: "ImportError: cannot import name 'Node'"**
- *Cause*: ROS 2 is not properly sourced
- *Solution*: Run `source /opt/ros/iron/setup.bash` (or your ROS 2 distro)

**Error 2: "RuntimeError: rclpy not initialized"**
- *Cause*: `rclpy.init()` not called before creating nodes
- *Solution*: Ensure `rclpy.init(args=args)` is called before creating any nodes

**Error 3: Node not showing in ros2 node list**
- *Cause*: Node name collision or node crashed after startup
- *Solution*: Check for unique node name and ensure node process is still running

**Error 4: "ModuleNotFoundError: No module named 'my_package.my_module'"**
- *Cause*: Package not built or not properly installed
- *Solution*: Run `colcon build` and `source install/setup.bash` in your workspace

**Error 5: "ros2: command not found"**
- *Cause*: ROS 2 environment is not sourced in current terminal
- *Solution*: Run `source /opt/ros/iron/setup.bash` (or your ROS 2 distro)

### Test Your Understanding

1. **What is the minimum code needed to create a ROS 2 node?**
   - Answer: Import rclpy and Node, create a class inheriting from Node, initialize rclpy, create the node instance, and call spin().

2. **What is the purpose of super().__init__('node_name') in a node class?**
   - Answer: This calls the parent Node class constructor, initializing the ROS 2 node with the provided name.

3. **What happens if rclpy.init() is not called before creating a node?**
   - Answer: A RuntimeError will occur since the ROS client library hasn't been initialized.

4. **Why is it important to call destroy_node() and rclpy.shutdown() during cleanup?**
   - Answer: It ensures proper cleanup of resources, prevents memory leaks, and allows for graceful shutdown of ROS communications.

## 1.4 Node Lifecycle and Management (400-500 words)

ROS 2 nodes have a defined lifecycle with distinct states that provide better control over system behavior. Understanding this lifecycle is crucial for building robust robotic systems that can handle initialization, runtime changes, and shutdown conditions properly.

### Node States
The standard ROS 2 node lifecycle consists of four primary states:

1. **Unconfigured**: The initial state when the node is created but not yet configured. In this state, the node exists but cannot participate in communication until it moves to the next state.

2. **Inactive**: The node is configured but not yet active. Publishers and subscribers exist but are not yet participating in communication. This state allows for parameter configuration before activation.

3. **Active**: The node is fully operational and participating in the ROS graph, with communication interfaces active and callbacks running.

4. **Finalized**: The node has been shut down and is no longer functional. All resources have been released.

Managed nodes (also called lifecycle nodes) provide explicit control over these states, which is particularly useful in complex robotic systems where coordinated startup and shutdown are important.

### Lifecycle Management Overview
Lifecycle nodes implement a state machine that can be controlled through services, allowing for coordinated startup and shutdown of complex systems. This is particularly important in systems where the order of operations matters, such as sensor initialization before processing nodes begin operation.

Lifecycle nodes must implement specific callback methods for each state transition:
- `on_configure()`: Called when transitioning from unconfigured to inactive
- `on_cleanup()`: Called when transitioning from inactive to unconfigured
- `on_activate()`: Called when transitioning from inactive to active
- `on_deactivate()`: Called when transitioning from active to inactive
- `on_shutdown()`: Called when transitioning to finalized from any state
- `on_error()`: Called when transitioning to finalized due to an error

### When to Use Managed Nodes vs Regular Nodes
Regular nodes are appropriate for:
- Simple applications where order of initialization is not critical
- Development and prototyping
- Standalone applications

Managed nodes are recommended for:
- Production systems requiring coordinated startup/shutdown
- Safety-critical applications
- Systems with complex initialization requirements
- Applications requiring dynamic reconfiguration

### Best Practices for Node Initialization
When designing nodes, consider these best practices:
- Initialize communication interfaces in the constructor
- Use parameters for configurable settings
- Implement proper error handling during initialization
- Validate parameters and resources before becoming active
- Plan for graceful degradation if optional resources are unavailable

### Resource Management and Cleanup
Proper resource management includes:
- Closing file handles and external connections
- Stopping timers and threads
- Releasing memory and communication resources
- Saving persistent state when appropriate

The destroy_node() method should be called explicitly to ensure proper cleanup, though it's also called automatically when the Node object is garbage collected.

### Code Example 3: Node with Parameter Declaration

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

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
        
        return SetParametersResult(successful=True)

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

# Required import for parameter callback return type
from rcl_interfaces.msg import SetParametersResult

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
```

## 1.5 Node Communication Overview (300-400 words)

Now that we understand the structure and lifecycle of nodes, let's look at how nodes communicate with each other. This communication forms the foundation of distributed robotic systems in ROS 2.

### Three Communication Patterns

ROS 2 provides three primary communication patterns that nodes use to interact:

1. **Topics (Publish/Subscribe)**: Asynchronous, one-way data flow from publishers to subscribers. This pattern is ideal for sensor data, status updates, and other continuous data streams. Publishers send messages without knowing who will receive them, and subscribers receive data without knowing the source.

2. **Services (Request/Response)**: Synchronous, bidirectional communication where a client sends a request and waits for a response from the server. This pattern is perfect for actions that require confirmation or computation with a clear result, like navigation goals or configuration changes.

3. **Actions**: Long-running tasks with feedback and goal management. Actions combine request/response with the ability to monitor progress, cancel requests, and provide intermediate feedback. They're ideal for complex behaviors like path planning or manipulation tasks.

### Quality of Service (QoS) Concept
Quality of Service settings allow nodes to specify communication requirements like reliability, durability, and history. For example, sensor data might use "best effort" delivery for performance, while critical commands might require "reliable" delivery with message persistence.

### Discovery Mechanism
ROS 2 nodes automatically discover each other through the DDS (Data Distribution Service) middleware. When a node starts, it announces itself on the network and discovers other nodes, allowing for dynamic system composition without hardcoding addresses or endpoints.

### Node Graph Visualization
The ROS 2 system maintains a graph of all active nodes and their connections. Tools like `rqt_graph` visualize this graph, showing how different nodes communicate:

```
[parameter_node] --(robot_status)--> [status_monitor]
    |
    | (robot_commands)
    v
[command_processor]
```

💡 **Pro Tip**: Use `ros2 run rqt_graph rqt_graph` to visualize your node network during development. This is invaluable for understanding system architecture and debugging communication issues.

⚠️ **Warning**: A common beginner mistake is assuming all nodes can communicate immediately upon startup. In practice, there's a discovery period before nodes establish connections, so don't be surprised if the first few messages don't reach subscribers.

## 1.6 Best Practices (300-400 words)

Following established best practices is essential for creating maintainable, efficient, and reliable ROS 2 nodes. These practices have emerged from years of experience building robotic systems.

### Naming Conventions
- Use descriptive, lowercase names with underscores: `sensor_processor`, `navigation_controller`
- Avoid generic names like `node1`, `main`
- Use consistent naming patterns across your system
- For parameters, use a consistent namespace: `sensor_range`, `control_gain`

### Single Responsibility Principle
Each node should have a clear, single purpose. For example, don't create a node that handles both sensor processing and actuator control. Instead, create separate nodes for each function and let them communicate through appropriate interfaces.

### Error Handling Strategies
- Always implement proper error handling in callbacks
- Use try-catch blocks for operations that might fail
- Log errors appropriately with context
- Design nodes to continue operation when possible even with partial failures

### Logging Best Practices
- Use appropriate logging levels (DEBUG, INFO, WARN, ERROR)
- Include context in log messages
- Avoid logging sensitive information
- Use logging for debugging and operational information, not for data transmission

```python
# Good example
self.get_logger().info(f'Connected to sensor {sensor_name} at IP {sensor_ip}')

# Less useful
self.get_logger().info('Connected to sensor')
```

### Performance Considerations
- Keep callbacks lightweight and fast
- Use threading for expensive operations if necessary
- Consider message frequency and network load
- Use appropriate QoS settings for your use case
- Be mindful of memory usage, especially in resource-constrained environments

### Testing and Debugging Tips
- Write nodes that are testable in isolation
- Use parameters to control behavior during testing
- Implement diagnostic publishers to report internal state
- Use ROS 2's built-in tools like `ros2 topic echo` for debugging
- Create unit tests for complex logic within nodes

💡 **Pro Tip**: When designing nodes, think about how they'll be tested. If a node is difficult to test, it might indicate that the responsibilities aren't well-separated.

## Hands-On Project: "Hello Robot Node" (200-300 words + code)

Let's create a practical mini-project that combines everything we've learned. This will be a "Hello Robot" node that publishes status messages and responds to parameters.

### Goal:
Build a node that publishes robot status messages at configurable intervals, demonstrating node creation, parameter usage, and publisher functionality.

### Requirements:
1. Node publishes status every configurable interval
2. Accepts robot name as a parameter
3. Includes proper shutdown handling
4. Uses logging appropriately

```python
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
```

### Running Instructions:
1. Create the file as `hello_robot_node.py` in your package
2. Build your package: `colcon build`
3. Source your workspace: `source install/setup.bash`
4. Run the node: `ros2 run my_first_ros2_pkg hello_robot_node`
5. In another terminal, listen to the messages: `ros2 topic echo /robot_status std_msgs/msg/String`
6. Try changing parameters: `ros2 run my_first_ros2_pkg hello_robot_node --ros-args -p robot_name:=Rover -p status_interval:=1.0`

## Troubleshooting Guide

### Common Debugging Techniques
- Use `ros2 node list` to see all active nodes
- Use `ros2 node info <node_name>` to get detailed information about a specific node
- Use `ros2 param list` to see all parameters
- Use `ros2 topic list` to see all active topics
- Use `ros2 topic echo <topic_name>` to view messages on a topic

### Common Beginner Pitfalls
1. **Forgetting to source the workspace**: Always source `install/setup.bash` after building
2. **Using the same node name**: Each node in the system must have a unique name
3. **Not initializing rclpy**: Always call `rclpy.init()` before creating nodes
4. **Not handling shutdown properly**: Always call `destroy_node()` and `rclpy.shutdown()`

### Environment Issues
- If you get "command not found" errors, ROS 2 is not sourced (run `source /opt/ros/iron/setup.bash`)
- If nodes aren't connecting, check if they're in the same ROS_DOMAIN_ID
- If you're having Python import issues, ensure your PYTHONPATH includes your workspace

### Version Compatibility
- Ensure all packages in your system are the same ROS 2 distribution
- Keep your code compatible with the target ROS 2 version
- Use `ros2 --version` to check your current version

## Chapter Summary

In this chapter, we've covered the fundamental concept of nodes in ROS 2 - the basic building blocks of any ROS 2 system. We explored how nodes are structured, created, and managed, including:

- The essential components of a ROS 2 node and how they work together
- How to create and run your first node with proper initialization and cleanup
- The lifecycle of nodes and when to use managed nodes vs regular nodes
- How nodes prepare for communication with other system components
- Best practices for naming, error handling, and performance

Nodes form the foundation of distributed robotic systems, each handling specific responsibilities while communicating with other nodes to achieve complex behaviors. Understanding nodes is crucial for working with the publish/subscribe and service-based communication patterns covered in the next sections.

The hands-on project demonstrated how to create a functional node that publishes status messages with configurable parameters, applying everything learned in this chapter.

## Additional Resources

- [Official ROS 2 Node Tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [ROS 2 Parameter Guide](https://docs.ros.org/en/rolling/How-To-Guides/Using-parameters-in-a-class-cpp.html)
- [ROS 2 Node Architecture](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS 2 Q&A Community](https://answers.ros.org/questions/)
- [Example Node Implementations](https://github.com/ros2/examples/tree/master/rclpy)

For video tutorials, check the ROS 2 official YouTube channel or look for "ROS 2 Nodes for Beginners" content from reputable robotics educators.