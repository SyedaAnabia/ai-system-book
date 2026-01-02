# Section 1: Understanding ROS 2 Nodes - Code Examples

This directory contains the code examples from Chapter 2, Section 1: "Understanding ROS 2 Nodes".

## Examples Included

### 1. minimal_node.py
A basic ROS 2 node with minimal functionality. Demonstrates the essential components of a ROS 2 node.

**To run:**
```bash
# Build your workspace first
cd ~/ros2_ws
colcon build --packages-select your_package_name
source install/setup.bash

# Run the node
ros2 run your_package_name minimal_node
```

### 2. first_node.py
A simple node that prints a message every 2 seconds. This is the "first node" created in the tutorial section.

**To run:**
```bash
ros2 run your_package_name first_node
```

### 3. complete_node.py
A more advanced node demonstrating:
- Multiple log levels
- Publisher functionality
- Parameter handling
- Error handling

**To run:**
```bash
ros2 run your_package_name complete_node --ros-args -p robot_name:=MyRobot
```

### 4. param_node.py
A node that demonstrates parameter declaration and dynamic parameter updates.

**To run:**
```bash
ros2 run your_package_name param_node
```

You can change parameters while the node is running:
```bash
# Change robot name
ros2 param set /parameter_node robot_name NewRobotName

# Change update rate
ros2 param set /parameter_node update_rate 3.0
```

### 5. hello_robot_node.py
A complete example node that publishes status messages with configurable parameters.

**To run:**
```bash
ros2 run your_package_name hello_robot_node
```

With custom parameters:
```bash
ros2 run your_package_name hello_robot_node --ros-args -p robot_name:=Rover -p status_interval:=1.0
```

## Package Setup

To use these examples, create a ROS 2 package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_ros2_pkg
```

Then copy the Python files to `~/ros2_ws/src/my_first_ros2_pkg/my_first_ros2_pkg/`.

Your package.xml should include:
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>rcl_interfaces</depend>
```

And your setup.py should include the entry points if you want to run the nodes directly:
```python
entry_points={
    'console_scripts': [
        'minimal_node = my_first_ros2_pkg.minimal_node:main',
        'first_node = my_first_ros2_pkg.first_node:main',
        'complete_node = my_first_ros2_pkg.complete_node:main',
        'param_node = my_first_ros2_pkg.param_node:main',
        'hello_robot_node = my_first_ros2_pkg.hello_robot_node:main',
    ],
},
```

## Troubleshooting

- If you get "command not found" errors, make sure you've sourced your workspace: `source ~/ros2_ws/install/setup.bash`
- If nodes don't appear in `ros2 node list`, check that your package was built correctly with `colcon build`
- If you get import errors, make sure your Python files have the correct structure and the package is built properly