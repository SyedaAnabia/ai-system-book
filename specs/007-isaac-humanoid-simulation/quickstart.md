# Quickstart Guide: Isaac Humanoid Simulation and Navigation System

## Prerequisites

Before starting with the Isaac Humanoid Simulation and Navigation System, ensure your system meets the following requirements:

### Hardware Requirements
- Linux x86_64 system with NVIDIA GPU (RTX series recommended)
- At least 16GB RAM (32GB recommended)
- NVIDIA GPU with compute capability 6.0 or higher
- At least 50GB free disk space for Isaac Sim installation and assets

### Software Requirements
- Ubuntu 22.04 LTS or later
- ROS 2 Humble Hawksbill
- NVIDIA GPU drivers (520.61.05 or later)
- CUDA 11.8 or later
- cuDNN 8.6 or later
- Isaac Sim 2023.1.0 or later
- Isaac ROS 3.0 or later

## Installation

### 1. Install ROS 2 Humble

```bash
# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 packages
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-ros-core
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install NVIDIA Isaac Sim

Download Isaac Sim from the NVIDIA Developer website and follow the installation instructions. The default installation path is typically `/home/$USER/.local/share/ov/pkg/isaac_sim-2023.1.1`.

If you're using a Docker setup:

```bash
# Pull the Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run Isaac Sim in Docker
xhost +local:docker
docker run --gpus all -it --rm --network=host --privileged \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$HOME/.Xauthority:/root/.Xauthority:rw \
  --volume=$HOME/isaac-sim-workspace:/workspace:rw \
  --env="DISPLAY=$DISPLAY" \
  --env="ACCEPT_EULA=Y" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### 3. Install Isaac ROS Components

```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-gems ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-nav2-bt ros-humble-isaac-ros-segmentation

# Install additional dependencies for perception
sudo apt install ros-humble-vision-msgs ros-humble-geometry-msgs
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2
```

## Getting Started

### 1. Clone the Isaac Humanoid Simulation Repository

```bash
# Create a ROS 2 workspace
mkdir -p ~/isaac_humanoid_ws/src
cd ~/isaac_humanoid_ws/src

# Clone the necessary repositories
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark.git
```

### 2. Build the Workspace

```bash
cd ~/isaac_humanoid_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch a Basic Humanoid Simulation

```bash
# Source the workspace
cd ~/isaac_humanoid_ws
source install/setup.bash

# Launch the Isaac Sim environment with a basic humanoid
ros2 launch isaac_ros_common isaac_sim.launch.py

# In another terminal, load a humanoid robot
# This command will load a basic humanoid model into the simulation
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/
python3 scripts/sample_scripts/humanoid_example.py
```

### 4. Configure Humanoid Robot in Isaac Sim

First, create a URDF file for your humanoid robot in `~/isaac_humanoid_ws/src/robot_description/urdf/humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Robot base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.25 0.15 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.45"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="torso_head_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Left leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="torso_left_thigh_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0 -0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.04"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_thigh_shin_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.04"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shin_foot_joint" type="fixed">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.2"/>
  </joint>

  <!-- Right leg (similar to left leg) -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_right_thigh_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0.1 0 -0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.04"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_thigh_shin_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.04"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_shin_foot_joint" type="fixed">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.2"/>
  </joint>

  <!-- RGB Camera sensor -->
  <sensor name="rgb_camera" type="camera">
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
    <parent link="head"/>
    <camera name="rgb">
      <image width="640" height="480" format="RGB8"/>
      <clip near="0.05" far="10"/>
      <horizontal_fov>1.047</horizontal_fov>
    </camera>
  </sensor>

  <!-- Depth Camera sensor -->
  <sensor name="depth_camera" type="depth">
    <origin xyz="0.12 0 0.1" rpy="0 0 0"/>
    <parent link="head"/>
    <camera name="depth">
      <image width="640" height="480"/>
      <clip near="0.05" far="10"/>
      <horizontal_fov>1.047</horizontal_fov>
    </camera>
  </sensor>
</robot>
```

### 5. Set up Nav2 for Humanoid Navigation

Create a launch file for the navigation system at `~/isaac_humanoid_ws/src/isaac_humanoid_nav/launch/navigation.launch.py`:

```python
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('isaac_humanoid_nav')
    
    # Launch Nav2
    nav2_bringup_launch = launch_ros.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    # Launch the path follower
    path_follower_node = launch_ros.actions.Node(
        package='isaac_humanoid_nav',
        executable='path_follower',
        name='path_follower',
        parameters=[
            os.path.join(pkg_share, 'config', 'humanoid_nav_params.yaml')
        ],
        remappings=[
            ('/cmd_vel', '/humanoid/cmd_vel'),
            ('/odom', '/humanoid/odom')
        ]
    )
    
    # Launch the SLAM node
    slam_node = launch_ros.actions.Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        parameters=[
            {'use_sim_time': True},
            {'enable_occupancy_map': True},
            {'occupancy_map_resolution': 0.05},
            {'occupancy_map_size': 25}
        ]
    )
    
    return launch.LaunchDescription([
        nav2_bringup_launch,
        path_follower_node,
        slam_node
    ])
```

### 6. Run a Complete Simulation Example

```bash
# Terminal 1: Start Isaac Sim
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/
python3 -m omni.isaac.kit --ext-folder exts --config=../isaac_ros_dev_exts
# Or if using Docker launch:
docker run --gpus all -it --rm --network=host --privileged \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$HOME/.Xauthority:/root/.Xauthority:rw \
  --volume=$HOME/isaac-sim-workspace:/workspace:rw \
  --env="DISPLAY=$DISPLAY" \
  --env="ACCEPT_EULA=Y" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  nvcr.io/nvidia/isaac-sim:2023.1.1

# Terminal 2: Launch the simulation
cd ~/isaac_humanoid_ws
source install/setup.bash
ros2 launch isaac_ros_common isaac_sim.launch.py

# Terminal 3: Load the humanoid robot and start navigation
cd ~/isaac_humanoid_ws
source install/setup.bash
# Load the robot into simulation
# In Isaac Sim UI: Create -> Prims -> Robot -> From File -> Select your URDF
# Or use the API to load:
python3 scripts/load_humanoid_robot.py

# Terminal 4: Start navigation
cd ~/isaac_humanoid_ws
source install/setup.bash
ros2 launch isaac_humanoid_nav navigation.launch.py

# Terminal 5: Send navigation goals
cd ~/isaac_humanoid_ws
source install/setup.bash
ros2 run nav2_msgs send_goal 1.0 1.0 0.0
```

## API Quickstart

The Isaac Humanoid Simulation and Navigation System provides a REST API for programmatic control. Here's how to interact with it:

### 1. Starting the API Server

```bash
cd ~/isaac_humanoid_ws
source install/setup.bash
ros2 run isaac_humanoid_api server --ros-args -p port:=8080
```

### 2. Creating an Environment

```bash
curl -X POST http://localhost:8080/environments \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Office Environment",
    "description": "A realistic office environment for humanoid navigation",
    "physicsProperties": {
      "gravity": {"x": 0, "y": 0, "z": -9.81},
      "frictionCoefficient": 0.5,
      "restitutionCoefficient": 0.2
    }
  }'
```

### 3. Spawning a Robot

```bash
# First, create a robot model
curl -X POST http://localhost:8080/robots \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Atlas-like Humanoid",
    "urdfPath": "/path/to/humanoid.urdf",
    "sensors": [
      {
        "name": "rgb_camera",
        "sensorType": "RGB_CAMERA",
        "configuration": {
          "resolution": {"width": 640, "height": 480},
          "fovHorizontal": 60,
          "frameRate": 30
        }
      }
    ]
  }'

# Then spawn the robot in an environment
curl -X POST http://localhost:8080/robots/{ROBOT_ID}/spawn \
  -H "Content-Type: application/json" \
  -d '{
    "environmentId": "{ENVIRONMENT_ID}",
    "startPosition": {"x": 0, "y": 0, "z": 0},
    "startOrientation": {"x": 0, "y": 0, "z": 0, "w": 1}
  }'
```

### 4. Planning and Executing Navigation

```bash
# Plan a navigation path
curl -X POST http://localhost:8080/navigation/plan \
  -H "Content-Type: application/json" \
  -d '{
    "robotId": "{ROBOT_ID}",
    "start": {"x": 0, "y": 0, "z": 0},
    "goal": {"x": 5, "y": 5, "z": 0},
    "environmentId": "{ENVIRONMENT_ID}"
  }'

# Execute the planned navigation
curl -X POST http://localhost:8080/navigation/execute \
  -H "Content-Type: application/json" \
  -d '{
    "planId": "{PLAN_ID}",
    "robotId": "{ROBOT_ID}"
  }'
```

## Troubleshooting

### Common Issues and Solutions

1. **Isaac Sim fails to start with OpenGL errors**
   - Make sure your NVIDIA drivers are up to date
   - Verify that your GPU supports the required OpenGL version
   - Try running Isaac Sim with the headless option if display issues persist

2. **Robot falls through the floor in simulation**
   - Check the mass and inertia values in your URDF
   - Ensure physics properties (friction and restitution) are set appropriately
   - Verify that collision geometry is properly defined

3. **SLAM algorithm fails to build consistent map**
   - Ensure proper sensor configuration with realistic noise models
   - Check that the robot has sufficient distinctive landmarks in view
   - Verify that the SLAM parameters are tuned for humanoid locomotion

4. **Navigation fails with "No valid path found"**
   - Check that Nav2 parameters are appropriate for humanoid morphology
   - Verify that the robot's geometric representation matches in path planner
   - Ensure the costmap is properly configured for bipedal locomotion

For more detailed troubleshooting, see the full documentation in the `docs/` folder of the project.