# Quickstart Guide: AI Systems in the Physical World - Embodied Intelligence

**Feature**: 001-ros2-book-spec
**Date**: 2025-12-09

This quickstart guide provides the essential steps to set up your environment for working with the examples in "AI Systems in the Physical World - Embodied Intelligence".

## Prerequisites

Before starting with the book's content, ensure you have the following:

### System Requirements
- Operating System: Ubuntu 22.04 LTS or Windows 10/11 with WSL2
- RAM: 8GB minimum, 16GB recommended
- Storage: 20GB free space for all environments
- GPU: NVIDIA GPU with CUDA support recommended for Isaac modules

### Software Requirements
- Git version control system
- Docker (for isolated environment setups)
- Node.js v18+ and npm (for Docusaurus documentation)
- Python 3.8+ with pip package manager
- Visual Studio Code (or similar IDE) with Python and Markdown extensions

## Setting Up the Book Environment

### 1. Clone the Book Repository
```bash
git clone https://github.com/[your-repo]/ai-systems-book.git
cd ai-systems-book
```

### 2. Install and Run the Documentation Site
```bash
# Install dependencies
npm install

# Start the development server
npm start
```

Your documentation site will be available at `http://localhost:3000`.

### 3. Set Up ROS 2 Environment (Module 1)
For Ubuntu 22.04:
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Install ROS 2 Iron
sudo apt update
sudo apt install -y ros-iron-desktop
sudo apt install -y python3-rosdep2

# Source ROS 2 environment
source /opt/ros/iron/setup.bash

# For convenience, add to your ~/.bashrc
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
```

For Windows users:
1. Install WSL2 with Ubuntu 22.04
2. Follow the Ubuntu setup instructions within WSL

### 4. Set Up Simulation Environments (Module 2)

#### Gazebo Garden
```bash
# On Ubuntu
sudo apt install ros-iron-gazebo-*
sudo apt install gazebo

# Test installation
gz sim -v
```

#### Unity (Optional - for specific examples)
1. Download Unity Hub from unity3d.com
2. Install Unity 2022.3 LTS version
3. Install the ROS# package for Unity if working with Unity examples

## Running Code Examples

### 1. Navigate to Book Examples
```bash
# The code examples are organized by module
cd static/code-examples/
ls -la
```

### 2. Example: Running a Basic ROS 2 Publisher-Subscriber
This example corresponds to Chapter 1 of Module 1.

```bash
# Terminal 1 - Start the publisher
cd static/code-examples/module1-ros2/basic-pub-sub/
python3 publisher.py

# Terminal 2 - Start the subscriber
cd static/code-examples/module1-ros2/basic-pub-sub/
python3 subscriber.py
```

### 3. Example: Running a Python ROS 2 Node
This example corresponds to Chapter 3 of Module 1.

```bash
# Make sure ROS 2 is sourced
source /opt/ros/iron/setup.bash

# Navigate to the example
cd static/code-examples/module1-ros2/python-rclpy/
python3 simple_node.py
```

## Testing Your Setup

### 1. Verify ROS 2 Installation
```bash
# Check ROS 2 version
ros2 --version

# List available commands
ros2

# Test basic ROS 2 functionality
ros2 topic list
```

### 2. Run a Simple Test Script
```bash
# Navigate to the setup tests directory
cd static/code-examples/setup-tests/
python3 verify_setup.py
```

This script will run a series of tests to verify that your environment is properly configured for the book's exercises.

## Troubleshooting

### Common Issues and Solutions

1. **ROS 2 commands not found**:
   - Ensure you sourced the ROS 2 environment: `source /opt/ros/iron/setup.bash`
   - Check that you installed ROS 2 properly and it matches your distribution

2. **Python packages not found**:
   - Install missing packages: `pip3 install <package-name>`
   - Make sure you're using Python 3.8+

3. **Gazebo not launching**:
   - Check graphics drivers, especially on Linux
   - For WSL users, ensure WSLg is properly configured

## Next Steps

Once your environment is set up, you're ready to begin with Module 1: The Robotic Nervous System (ROS 2). Start with Chapter 1: Introduction to ROS 2 Architecture to get familiar with the fundamental concepts.

Remember to check the prerequisites for each chapter before proceeding, as later chapters build upon concepts and installations from earlier ones.