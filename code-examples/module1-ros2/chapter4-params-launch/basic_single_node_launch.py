# basic_single_node_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_parameter_server',  # Using our custom package
            executable='simple_parameter_server',  # Using our custom executable
            name='parameter_demo_node',
            parameters=[
                {'robot_name': 'launch_robot'},
                {'operational_mode': 'demo'},
                {'max_velocity': 1.5}
            ],
            output='screen'
        )
    ])