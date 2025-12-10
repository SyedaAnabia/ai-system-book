# Introduction to ROS 2 Architecture

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain the fundamental concepts of ROS 2 architecture including nodes, topics, services, and actions
2. Identify the key differences between ROS 1 and ROS 2
3. Understand how DDS middleware facilitates communication in ROS 2 systems
4. Install ROS 2 on different operating systems (Ubuntu, Windows, macOS)
5. Create and run a basic publisher-subscriber application in ROS 2

## Prerequisites

Before starting this chapter, you should have:
- Basic programming knowledge in Python or C++
- Familiarity with distributed systems concepts
- Experience with command-line interfaces
- Understanding of basic robotics concepts (optional but helpful)

## Estimated Reading Time

This chapter should take approximately 50-60 minutes to complete, including hands-on practice with the code examples.

## Introduction

Welcome to the first chapter of "AI Systems in the Physical World - Embodied Intelligence". In this chapter, we'll explore the fundamentals of ROS 2 (Robot Operating System 2), the middleware that powers countless robotic applications worldwide. ROS 2 is not a traditional operating system but rather a collection of tools, libraries, and conventions that provide a framework for developing robotic applications.

This chapter will provide you with a solid foundation in ROS 2's architecture, helping you understand how nodes communicate, how middleware facilitates this communication, and why ROS 2 represents a significant evolution from its predecessor. As we progress through this chapter, you'll learn to install ROS 2, write your first ROS 2 program, and understand the concepts that will form the building blocks for the entire course.

Understanding ROS 2 architecture is crucial as it underpins all other concepts in this book. By the end of this chapter, you'll have a working ROS 2 environment and a simple publisher-subscriber application running.

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robotic software applications that provides libraries and tools to help create robotic applications. It's important to clarify that ROS 2 is not an operating system in the traditional sense but rather a middleware framework that runs on top of existing operating systems like Linux, Windows, or macOS.

### Middleware Architecture

At its core, ROS 2 is built around a middleware layer that abstracts the complexities of inter-process communication. This middleware enables different software components, called "nodes," to communicate with each other regardless of the programming language they're written in, the operating system they're running on, or even the physical location of these nodes on a network.

ROS 2 uses Data Distribution Service (DDS) as its default middleware. DDS is an open standard for distributed systems that provides discovery, serialization, and delivery of data between applications. This DDS-based architecture allows ROS 2 to support large-scale distributed robotic systems with stringent requirements for real-time performance and reliability.

### Nodes

The fundamental building block of any ROS 2 system is the "node." A node is a single process that performs computation. Nodes are organized into packages to provide a modular structure to the software. Nodes communicate with each other through topics (for streaming data), services (for request-response communication), and actions (for goal-based communication with feedback).

In a typical robotic system, you might have nodes for:
- Sensor drivers that publish data from cameras, lidars, or IMUs
- Control nodes that send commands to actuators
- Perception nodes that process sensor data to understand the environment
- Planning nodes that determine navigation paths
- Behavioral nodes that coordinate complex tasks

### Communication

ROS 2's communication model is based on a distributed system where nodes can run on the same machine or different machines. The framework provides several communication patterns:

- **Publish/Subscribe**: Nodes publish messages to topics, which are then received by multiple subscriber nodes. This pattern is ideal for data that flows from one source to multiple consumers.

- **Service/Client**: Nodes can offer services that other nodes can request. This follows a request-response pattern where a client sends a request and receives a response.

- **Actions**: For long-running tasks that require goal setting, feedback, and result reporting, ROS 2 provides action servers and clients.

### Advantages of ROS 2

ROS 2 offers several key advantages over its predecessor:

- **Real-time capabilities**: ROS 2 supports real-time systems, making it suitable for safety-critical applications.
- **Multi-robot systems**: Better support for coordinating multiple robots working together.
- **Security**: Built-in security features to protect robotic systems from unauthorized access.
- **Quality of Service (QoS)**: Configurable communication policies to meet specific performance requirements.
- **Cross-platform support**: Runs on Linux, Windows, and macOS with consistent behavior.

ROS 2's design philosophy emphasizes code reuse, collaborative development, and rapid prototyping. The ecosystem includes numerous pre-built packages for common robotic tasks, extensive documentation, and a supportive community. This makes ROS 2 an ideal platform for both academic research and industrial applications.

## ROS 2 vs ROS 1

ROS 2 represents a significant architectural shift from ROS 1, addressing many of the fundamental limitations of the original framework. Understanding these differences is crucial for anyone transitioning from ROS 1 or starting fresh with ROS 2.

### Architectural Foundation

ROS 1 used a centralized architecture with a master node responsible for naming and registration. This created a single point of failure and limited scalability. ROS 2, in contrast, uses a decentralized architecture based on DDS (Data Distribution Service) middleware. This eliminates the need for a master node, allowing nodes to discover each other directly and enabling more robust and scalable systems.

### Real-time Support

One of the most significant improvements in ROS 2 is native real-time support. ROS 1 was not designed with real-time requirements in mind, making it unsuitable for applications with strict timing constraints. ROS 2 addresses this by providing real-time safe libraries and supporting real-time operating systems and kernel configurations. This makes ROS 2 suitable for safety-critical applications, industrial automation, and scenarios where deterministic behavior is essential.

### Multi-Robot Systems

ROS 1 had limited native support for multi-robot systems, often requiring complex network configurations or custom solutions to coordinate multiple robots. ROS 2 simplifies multi-robot development with its DDS-based discovery mechanism, allowing nodes from different robots to automatically discover and communicate with each other over a network without requiring central coordination.

### Security

Security was an afterthought in ROS 1, making robotic systems vulnerable to unauthorized access. ROS 2 includes built-in security features that support authentication, access control, and data encryption. These features are essential as robots become more connected and autonomous, requiring robust protection against cyber threats.

### Quality of Service (QoS)

ROS 2 introduces Quality of Service profiles that allow users to configure communication behavior based on their application's requirements. This includes settings for reliability, durability, deadline, and liveliness. These QoS settings enable ROS 2 to handle diverse application needs, from best-effort communication for sensor data to reliable delivery for critical commands.

### Programming Language Support

While ROS 1 primarily supported C++ and Python, ROS 2 has expanded language support. The DDS middleware allows for easy integration with additional programming languages, and there are now official ROS 2 clients for C++, Python, and RCL (Robot Client Libraries) implementations for other languages like Rust, Java, and C#.

### Installation and Dependency Management

ROS 2 uses standard package managers and build systems, making it more compatible with existing software development workflows. It supports both colcon (the default build tool) and standard CMake, as well as Python packaging tools, which eases integration with other software ecosystems.

## DDS Middleware

Data Distribution Service (DDS) is the middleware standard that underlies ROS 2's communication architecture. Understanding DDS is crucial to fully appreciate ROS 2's capabilities and how it differs from ROS 1.

### What is DDS?

DDS stands for Data Distribution Service and is an Object Management Group (OMG) standard for real-time, distributed data exchange. It's designed to efficiently connect data sources to data sinks with minimal latency and maximum reliability. DDS is particularly relevant to robotics because it addresses the challenges of distributed systems, real-time communication, and scalable architectures that are common in robotic applications.

### Key DDS Concepts

#### Data-Centric Architecture
DDS uses a data-centric approach rather than a message-centric one. In a message-centric system, communication happens through explicit message passing between specific endpoints. In a data-centric system, applications interact with data samples in a global "data space." When a data sample is published, DDS automatically delivers it to all interested subscribers based on the data's content and attributes, rather than requiring the publisher to know all subscribers in advance.

#### Global Data Space
DDS creates a virtual shared memory space where data is published and consumed. This global data space is distributed across all participating nodes in the system. Publishers write data to this space, and DDS ensures that the data is delivered to all subscribers with matching subscriptions. The global data space is transparent to applications, requiring no explicit management from developers.

#### Quality of Service (QoS) Policies
DDS provides a rich set of Quality of Service policies that allow developers to fine-tune communication behavior. These policies control aspects like reliability, durability, deadline, latency budget, and data persistence. QoS policies are negotiated between publishers and subscribers, ensuring that communication meets the requirements of the application.

### DDS in ROS 2

ROS 2 leverages DDS's capabilities to provide its communication primitives:

#### Topics
In ROS 2, topics map directly to DDS Topics. When a ROS 2 node publishes to a topic, it's actually publishing to a DDS Topic. The DDS infrastructure handles the discovery of subscribers and the delivery of messages. This allows for complex communication topologies where publishers and subscribers don't need to know about each other.

#### Services
ROS 2 services use DDS for request-response communication. Each service creates a pair of DDS Topics—one for requests and one for responses—which DDS manages to ensure reliable delivery and matching of requests to responses.

#### Actions
Actions in ROS 2 are implemented using multiple DDS Topics and provide goal-oriented communication with feedback and result delivery.

### Benefits of DDS in Robotics

#### Real-time Performance
DDS is designed for real-time systems and provides mechanisms to achieve deterministic communication behavior. This is essential for robotic applications that require precise timing, such as control loops or sensor processing.

#### Scalability
DDS can scale from embedded systems to large distributed networks. This makes it suitable for both small robots and large multi-robot systems.

#### Language and Platform Independence
DDS supports multiple programming languages and platforms, which is important in robotics where different components might be implemented in different languages or run on different hardware platforms.

#### Interoperability
Multiple DDS implementations exist from different vendors, all conforming to the same standard. This ensures that ROS 2 can work with various DDS implementations and that different DDS-based systems can interoperate.

## Installation Guide

Installing ROS 2 requires setting up the appropriate repositories and packages for your operating system. We'll cover the installation process for the three main operating systems: Ubuntu, Windows, and macOS.

### Ubuntu Installation

The following instructions are for Ubuntu 22.04 (Jammy Jellyfish), which is the most common development environment for ROS 2. The current LTS distribution is Iron Irwini, which will be supported until May 2025.

#### Setup Locale
Make sure your locale is set to UTF-8:

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

#### Setup Sources
Add the ROS 2 GPG key and repository:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Install ROS 2 Packages
Update package lists and install ROS 2:

```bash
sudo apt update
sudo apt install ros-iron-desktop
```

#### Environment Setup
Set up your environment to use ROS 2:

```bash
source /opt/ros/iron/setup.bash
```

To automatically source the ROS 2 environment in new terminals, add the following line to your `~/.bashrc`:

```bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
```

#### Install Additional Tools
Install tools for development:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

Initialize rosdep:

```bash
sudo rosdep init
rosdep update
```

### Windows Installation (WSL2)

Installing ROS 2 on Windows is best accomplished using Windows Subsystem for Linux (WSL2), which provides a Linux environment with full system call compatibility.

#### Install WSL2
First, install WSL2 and Ubuntu 22.04 from the Microsoft Store or using PowerShell:

```powershell
wsl --install Ubuntu-22.04
```

After installation, launch Ubuntu from the Start menu and follow the Ubuntu installation steps above.

#### Configure WSL2 for ROS 2
For proper ROS 2 functionality with GUI applications, install an X-server like VcXsrv and configure display:

1. Install VcXsrv Windows X Server
2. Start VcXsrv with the following settings:
   - Display settings: 0
   - Check "Disable access control"
   - Check "Native opengl" (if needed for graphics)

Then set your display in WSL2:

```bash
export DISPLAY=:0
```

#### Windows Installation Alternative
For native Windows installation, follow the official ROS 2 Windows installation guide, but note that this method is more complex and WSL2 is generally recommended for Windows users.

### macOS Installation

ROS 2 on macOS requires Homebrew and some additional components. The following steps will set up ROS 2 Iron on macOS.

#### Install Homebrew
If not already installed:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

#### Install Dependencies
Install required dependencies:

```bash
brew install cmake cppcheck eigen libomp opencv openssh openssl python3 rcutils tinyxml2 uncrustify
pip3 install -U argcomplete flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes lizard pytest pytest-cov pytest-repeat pytest-rerunfailures
```

#### Install Python and Tools
Install colcon and rosdep:

```bash
pip3 install -U colcon-common-extensions vcstool
pip3 install -U rosdep
```

#### Set Up OpenCV
macOS might have issues with OpenCV, so ensure it's properly linked:

```bash
brew install opencv
export OpenCV_DIR=$(brew --prefix opencv)/lib/cmake/opencv4
```

#### Clone and Build ROS 2
Download the ROS 2 source:

```bash
mkdir -p ~/ros2_iron/src
cd ~/ros2_iron
wget https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos
vcs import src < ros2.repos
```

Build ROS 2:

```bash
colcon build --merge-install --packages-skip-by-dep python_qt_binding rqt_gui_cpp rqt_py_common
```

Set up the environment:

```bash
source ~/ros2_iron/install/setup.sh
```

## First ROS 2 Program

Now that ROS 2 is installed, let's create your first ROS 2 program - a simple publisher-subscriber application that demonstrates the basic communication pattern in ROS 2.

### File Structure

First, create a workspace for your ROS 2 project:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Create a package for our example:

```bash
ros2 pkg create --build-type ament_python py_pubsub
```

### Publisher Code

Navigate to the Python directory in your new package and create a publisher script:

```bash
cd py_pubsub/py_pubsub
```

Create the publisher file `talker.py`:

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
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Publisher Code Explanation

Let's break down this publisher code line by line:

1. **Imports**: The code imports `rclpy` (the Python client library for ROS 2), the `Node` class, and the `String` message type from the standard message library.

2. **Class Definition**: `MinimalPublisher` inherits from `Node`, which is the base class for all ROS 2 nodes.

3. **Constructor (`__init__`)**: 
   - `super().__init__('minimal_publisher')` creates the node with the name 'minimal_publisher'
   - `self.create_publisher(String, 'topic', 10)` creates a publisher that sends `String` messages to the topic named 'topic' with a queue size of 10
   - `self.create_timer(timer_period, self.timer_callback)` creates a timer that calls `timer_callback` every 0.5 seconds
   - `self.i = 0` initializes a counter to include in the published messages

4. **Timer Callback**:
   - Creates a new `String` message
   - Sets `msg.data` to include the counter value
   - Publishes the message using the publisher
   - Logs the published message to the console
   - Increments the counter

5. **Main Function**:
   - Initializes the ROS 2 communication system
   - Creates an instance of the `MinimalPublisher`
   - Starts spinning to process callbacks (publishing messages)
   - Cleans up when done

### Subscriber Code

Now let's create a subscriber to receive the messages from the publisher. Create `listener.py` in the same directory:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Code Explanation

1. **Constructor**: Creates a subscription to the 'topic' with the same `String` message type and queue size of 10. The callback function `listener_callback` is called whenever a message is received.

2. **Listener Callback**: Receives the message and logs its content to the console.

### Setup Configuration

To make our scripts executable, update the package's `setup.py` file:

```bash
cd ~/ros2_ws/src/py_pubsub
```

Edit the `setup.py` file to include the following entry points:

```python
import os
from glob import glob
from setuptools import setup

from setuptools import find_packages

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.talker:main',
            'listener = py_pubsub.listener:main',
        ],
    },
)
```

### Build and Run

First, build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

Source the setup file:

```bash
source install/setup.bash
```

Now open two separate terminals and run:

Terminal 1 (Publisher):
```bash
ros2 run py_pubsub talker
```

Terminal 2 (Subscriber):
```bash
ros2 run py_pubsub listener
```

### Expected Output

You should see output similar to:

Publisher terminal:
```
[INFO] [1692027347.502208200] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1692027348.002441500] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1692027348.502725400] [minimal_publisher]: Publishing: "Hello World: 2"
```

Subscriber terminal:
```
[INFO] [1692027347.502301700] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [1692027348.002523800] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [1692027348.502790200] [minimal_subscriber]: I heard: "Hello World: 2"
```

### Understanding the Communication

This example demonstrates the publish/subscribe communication pattern in ROS 2:
- The publisher (`talker`) sends messages to a topic named 'topic'
- The subscriber (`listener`) listens to the same topic
- DDS middleware automatically handles the discovery and message delivery between nodes
- Multiple subscribers can listen to the same topic without changing the publisher code

## Exercises

### Exercise 1: Modify the Publisher Message
1. Modify the publisher to send a different message format that includes the current time
2. Add a parameter to control the publishing frequency
3. Test with different frequencies (0.1 seconds, 1 second, 2 seconds)

### Exercise 2: Create a Custom Message
1. Create a new message type with multiple fields (e.g., name, age, and active status)
2. Modify the publisher to send this new message type
3. Update the subscriber to receive and process the new message type

### Exercise 3: Add a Service
1. Create a service that allows clients to request the current count from the publisher
2. Implement a service client that can request and display the current count
3. Test the communication between the publisher (service server) and the client

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2 architecture. We began by understanding what ROS 2 is - a middleware framework that enables distributed robotic applications. We explored the key architectural components including nodes, which represent individual processes, and the various communication patterns ROS 2 provides: topics for streaming data, services for request-response communication, and actions for goal-based interactions.

We examined how ROS 2 differs significantly from ROS 1, particularly in its use of DDS middleware for decentralized communication, real-time support, enhanced security features, and better multi-robot system support. The DDS middleware provides a data-centric approach to communication, offering Quality of Service policies that allow fine-tuning of communication behavior based on application requirements.

The installation process for different operating systems was detailed, with special attention to Ubuntu (the most common development environment), Windows (using WSL2), and macOS. Finally, we implemented a complete ROS 2 publisher-subscriber example, demonstrating how nodes can communicate in a distributed system.

These foundational concepts are essential for understanding the more advanced topics in the subsequent chapters, where we'll explore specific ROS 2 components in greater depth.

## Further Reading

1. **ROS 2 Documentation**: The official ROS 2 documentation provides comprehensive guides and tutorials for all aspects of ROS 2 development. [https://docs.ros.org/en/rolling/](https://docs.ros.org/en/rolling/)

2. **DDS Specification**: The Object Management Group's official DDS specification provides detailed technical information about the middleware standard that powers ROS 2. [https://www.omg.org/spec/DDS/](https://www.omg.org/spec/DDS/)

3. **ROS 2 Design Papers**: Research papers describing the design decisions and architecture of ROS 2 provide deep insights into the framework's evolution from ROS 1. Look for papers from the Open Robotics team.

4. **Robotics Middleware Comparison**: Studies comparing different robotics middleware frameworks can help understand when to choose ROS 2 over alternatives like YARP, Ice, or custom solutions.

5. **Practical ROS 2 Examples**: Code repositories with real-world ROS 2 examples demonstrate how the concepts translate to practical applications in robotics research and production systems.