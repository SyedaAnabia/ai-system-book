# multi_node_launch_with_params.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Parameter server node
        Node(
            package='simple_parameter_server',
            executable='simple_parameter_server',
            name='param_server_1',
            parameters=[
                {'robot_name': 'robot1'},
                {'operational_mode': 'autonomous'},
                {'max_velocity': 1.0},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # Second parameter server node
        Node(
            package='simple_parameter_server',
            executable='simple_parameter_server',
            name='param_server_2',
            parameters=[
                {'robot_name': 'robot2'},
                {'operational_mode': 'teleop'},
                {'max_velocity': 0.5},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ])