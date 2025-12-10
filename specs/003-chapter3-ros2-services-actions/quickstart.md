# Quickstart Guide: Chapter 3 - ROS 2 Services and Actions

**Feature**: 003-chapter3-ros2-services-actions
**Date**: 2025-12-09

This quickstart guide provides the essential prerequisites and setup steps needed to work with the examples in Chapter 3: "ROS 2 Services and Actions" of the AI Systems in the Physical World book.

## Prerequisites

Before starting with Chapter 3 content, ensure you have completed Chapters 1 and 2 and have the following:

### System Requirements
- Operating System: Ubuntu 22.04 LTS, Windows 10/11 with WSL2, or macOS 12+
- RAM: 8GB minimum, 16GB recommended
- Storage: 5GB free space for ROS 2 Iron and examples
- Python 3.8+ with pip package manager

### Software Requirements
- ROS 2 Iron/Humble installed and properly sourced
- Python 3.8+ with pip
- Git version control system
- Visual Studio Code (or similar IDE) with Python extensions
- Basic command-line familiarity

### Chapter Prerequisites
- Understanding of basic ROS 2 concepts from Chapter 1 (nodes, basic architecture)
- Understanding of topic communication from Chapter 2 (publishers/subscribers)
- Ability to run simple ROS 2 commands
- Basic Python programming knowledge

## Setting Up for Chapter 3

### 1. Verify Your ROS 2 Installation
```bash
# Check ROS 2 version
ros2 --version

# Verify ROS 2 is sourced
echo $ROS_DISTRO
# Should output "iron" or "humble"

# List available commands
ros2
```

### 2. Create a Workspace for Chapter 3 Examples
```bash
# Create a workspace for examples
mkdir -p ~/chapter3_ws/src
cd ~/chapter3_ws

# Source ROS 2 environment
source /opt/ros/iron/setup.bash  # or humble
```

### 3. Download Chapter Examples
```bash
# Navigate to your book examples directory
cd ~/chapter3_ws/src

# If you have the examples in a repository, clone it:
# git clone https://github.com/[your-repo]/chapter3-examples.git

# Otherwise, you'll create examples as you go through the chapter
```

## Running Code Examples

### 1. Navigate to Chapter Examples
```bash
# The code examples are organized by section
cd ~/chapter3_ws/src/
ls -la
```

### 2. Example: Running a Simple Service
This example corresponds to Section 2 of the chapter.

```bash
# Create the directory for this example
mkdir -p ~/chapter3_ws/src/simple_service
cd ~/chapter3_ws/src/simple_service

# Create the Python package structure
mkdir simple_service
touch simple_service/__init__.py
touch setup.py
```

Create `simple_service/simple_service/add_two_ints_server.py`:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_server = AddTwoIntsServer()

    rclpy.spin(add_two_ints_server)

    add_two_ints_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Build and Run the Service Example
```bash
# From the workspace root
cd ~/chapter3_ws

# Build the package
colcon build --packages-select simple_service

# Source the workspace
source install/setup.bash

# Run the service server
ros2 run simple_service add_two_ints_server
```

### 4. Test the Service from Another Terminal
In a new terminal:
```bash
# Source ROS 2 and the workspace
source /opt/ros/iron/setup.bash  # or humble
source ~/chapter3_ws/install/setup.bash

# Call the service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### 5. Example: Running an Action
Create `simple_action/simple_action/fibonacci_action_server.py`:
```python
from action_tutorials_interfaces.action import Fibonacci
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(fibonacci_action_server, executor=executor)

    fibonacci_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Essential ROS 2 Commands for Chapter 3

### Service Commands
```bash
# List all available services
ros2 service list

# Call a service from command line (for testing)
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Get info about a service
ros2 service info /add_two_ints

# Get detailed info about a service type
ros2 service type /add_two_ints
```

### Action Commands
```bash
# List all available actions
ros2 action list

# Send a goal to an action server
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# Get info about an action
ros2 action info /fibonacci

# Get detailed info about an action type
ros2 action type /fibonacci
```

## Testing Your Setup

### 1. Verify Service Creation
```bash
# Terminal 1: Run the service server
source ~/chapter3_ws/install/setup.bash
ros2 run simple_service add_two_ints_server

# Terminal 2: List services to see the server
source ~/chapter3_ws/install/setup.bash
ros2 service list
# Should show "/add_two_ints"
```

### 2. Verify Service Communication
```bash
# Terminal 1: Service server (from above)

# Terminal 2: Test the service
source ~/chapter3_ws/install/setup.bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
# Should return a sum of 30
```

### 3. Verify Action Communication (after learning Section 4)
```bash
# Terminal 1: Action server

# Terminal 2: Action client
# ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

## Troubleshooting Common Issues

### Common Issue 1: Command Not Found
If you get "command not found" errors:
- Ensure ROS 2 is properly sourced: `source /opt/ros/iron/setup.bash` (or humble)
- Check your ROS_DISTRO environment variable: `echo $ROS_DISTRO`

### Common Issue 2: Service/Action Not Found
If services or actions don't show up:
- Check that the server node is running
- Verify service/action names match exactly
- Ensure both nodes are in the same ROS domain (or properly configured for different domains)

### Common Issue 3: Import Errors
If Python import statements fail:
- Verify ROS 2 is sourced before starting Python processes
- Make sure your workspace is built with `colcon build`
- Ensure you source the workspace: `source install/setup.bash`

### Common Issue 4: "ModuleNotFoundError: No module named 'example_interfaces'"
- This means the required interfaces package is not installed
- Install it with: `sudo apt install ros-iron-example-interfaces` (or humble equivalent)

## Next Steps

Once your environment is set up, you're ready to begin with Chapter 3: "ROS 2 Services and Actions". Start with the Introduction section to get familiar with the fundamental concepts before moving to service and action implementations.

Remember to test each code example as you encounter it in the chapter, and don't hesitate to experiment with the examples to deepen your understanding of ROS 2 communication patterns.