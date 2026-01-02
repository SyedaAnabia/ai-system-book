# launch_with_arguments.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Declare launch arguments
    robot_model = LaunchConfiguration('robot_model')
    enable_logging = LaunchConfiguration('enable_logging')
    simulation_mode = LaunchConfiguration('simulation_mode')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_model',
            default_value='nao_v6',
            description='Robot model to use for the launch'
        ),
        
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            choices=['true', 'false'],
            description='Enable detailed logging for debugging'
        ),
        
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='false',
            choices=['true', 'false'],
            description='Launch in simulation mode with Gazebo'
        ),
        
        # Log the selected robot model
        LogInfo(
            msg=['Launching for robot model: ', robot_model]
        ),
        
        # Conditionally launch Gazebo if in simulation mode
        Node(
            package='gazebo_ros',
            executable='gazebo',
            condition=IfCondition(simulation_mode),
            arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Main robot controller with argument-based parameters
        Node(
            package='humanoid_controller',
            executable='controller_node',
            name=['controller_', robot_model],
            parameters=[
                {'robot_model': robot_model},
                {'use_sim_time': simulation_mode},
                {'enable_detailed_logging': enable_logging},
                {'control_frequency': 50.0 if PythonExpression([simulation_mode, ' == true']) else 100.0}
            ],
            output='both' if enable_logging else 'log'
        ),
        
        # Diagnostic aggregator based on robot model
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name=['diag_agg_', robot_model],
            parameters=[
                {'robot_model': robot_model},
                {'use_sim_time': simulation_mode}
            ],
            remappings=[
                ('/diagnostics', [robot_model, '/diagnostics']),
                ('/diagnostics_agg', [robot_model, '/diagnostics_agg'])
            ]
        )
    ])