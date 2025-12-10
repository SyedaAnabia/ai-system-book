# Quickstart Guide: Chapter 2 - Nodes, Topics, and Services

**Feature**: 002-chapter2-nodes-topics-services
**Date**: 2025-12-09

This quickstart guide provides the essential prerequisites and setup steps needed to work with the examples in Chapter 2: "Nodes, Topics, and Services" of the AI Systems in the Physical World book.

## Prerequisites

Before starting with Chapter 2 content, ensure you have completed Chapter 1 and have the following:

### System Requirements
- Operating System: Ubuntu 22.04 LTS, Windows 10/11 with WSL2, or macOS 12+
- RAM: 8GB minimum, 16GB recommended
- Storage: 5GB free space for ROS 2 Iron and examples
- Python 3.8+ with pip package manager

### Software Requirements
- ROS 2 Iron/Iguana installed and properly sourced
- Python 3.8+ with pip
- Git version control system
- Visual Studio Code (or similar IDE) with Python extensions
- Basic command-line familiarity

### Chapter 1 Prerequisites
- Understanding of basic ROS 2 concepts from Chapter 1
- Working ROS 2 environment
- Ability to run simple ROS 2 commands
- Basic Python programming knowledge

## Setting Up for Chapter 2

### 1. Verify Your ROS 2 Installation
```bash
# Check ROS 2 version
ros2 --version

# Verify ROS 2 is sourced
echo $ROS_DISTRO
# Should output "iron" or "iguanav2"

# List available commands
ros2
```

### 2. Create a Workspace for Chapter 2 Examples
```bash
# Create a workspace for examples
mkdir -p ~/chapter2_ws/src
cd ~/chapter2_ws

# Source ROS 2 environment
source /opt/ros/iron/setup.bash
```

### 3. Download Chapter Examples
```bash
# Navigate to your book examples directory
cd ~/chapter2_ws/src

# If you have the examples in a repository, clone it:
# git clone https://github.com/[your-repo]/chapter2-examples.git

# Otherwise, you'll create examples as you go through the chapter
```

## Running Code Examples

### 1. Navigate to Chapter Examples
```bash
# The code examples are organized by section
cd ~/chapter2_ws/src/
ls -la
```

### 2. Example: Running a Node Publisher
This example corresponds to Section 2.3 of the chapter.

```bash
# Create the directory for this example
mkdir -p ~/chapter2_ws/src/basic_publisher
cd ~/chapter2_ws/src/basic_publisher

# Create the Python package structure
mkdir basic_publisher
touch basic_publisher/__init__.py
touch setup.py
```

Create `basic_publisher/basic_publisher/publisher_member_function.py`:
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Build and Run the Example
```bash
# From the workspace root
cd ~/chapter2_ws

# Build the package
colcon build --packages-select basic_publisher

# Source the workspace
source install/setup.bash

# Run the publisher
ros2 run basic_publisher publisher_member_function
```

### 4. Test with Subscriber in Another Terminal
In a new terminal:
```bash
# Source ROS 2 and the workspace
source /opt/ros/iron/setup.bash
source ~/chapter2_ws/install/setup.bash

# Run a subscriber to test communication
ros2 topic echo /topic std_msgs/msg/String
```

## Essential ROS 2 Commands for Chapter 2

### Node Commands
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info <node_name>

# Execute commands in a namespace (if applicable)
ros2 run <package_name> <executable_name> --ros-args --remap __ns:=/namespace
```

### Topic Commands
```bash
# List all active topics
ros2 topic list

# Echo messages from a topic (view messages being published)
ros2 topic echo <topic_name> <msg_type>

# Publish a message from command line (for testing)
ros2 topic pub <topic_name> <msg_type> <values>

# Get info about a topic (who is publishing/subscribing)
ros2 topic info <topic_name>

# Check the rate of messages on a topic
ros2 topic hz <topic_name>
```

### Service Commands
```bash
# List all available services
ros2 service list

# Call a service from command line
ros2 service call <service_name> <service_type> <request_values>

# Get info about a service
ros2 service info <service_name>
```

## Testing Your Setup

### 1. Verify Node Creation
```bash
# Terminal 1: Run a simple publisher
source ~/chapter2_ws/install/setup.bash
ros2 run basic_publisher publisher_member_function

# Terminal 2: List nodes to see the publisher
source ~/chapter2_ws/install/setup.bash
ros2 node list
# Should show "minimal_publisher"
```

### 2. Verify Topic Communication
```bash
# Terminal 1: Publisher (from above)

# Terminal 2: Subscriber
source ~/chapter2_ws/install/setup.bash
ros2 topic echo /topic std_msgs/msg/String

# You should see messages from the publisher in the subscriber terminal
```

### 3. Verify Service Communication (after learning Section 3)
```bash
# Terminal 1: Service server

# Terminal 2: Service client
# ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

## Troubleshooting Common Issues

### Common Issue 1: Command Not Found
If you get "command not found" errors:
- Ensure ROS 2 is properly sourced: `source /opt/ros/iron/setup.bash`
- Check your ROS_DISTRO environment variable: `echo $ROS_DISTRO`

### Common Issue 2: No Messages Received
If topics don't show communication:
- Check that both publisher and subscriber are using the same topic name
- Verify QoS profiles are compatible between publisher and subscriber
- Ensure both nodes are in the same ROS domain (or properly configured for different domains)

### Common Issue 3: Import Errors
If Python import statements fail:
- Verify ROS 2 is sourced before starting Python processes
- Make sure your workspace is built with `colcon build`
- Ensure you source the workspace: `source install/setup.bash`

## Next Steps

Once your environment is set up, you're ready to begin with Chapter 2: "Nodes, Topics, and Services". Start with Section 1: "Understanding ROS 2 Nodes" to get familiar with the fundamental concepts before moving to communication patterns.

Remember to test each code example as you encounter it in the chapter, and don't hesitate to experiment with the examples to deepen your understanding of ROS 2 communication patterns.