# conditional_launching.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Declare launch arguments
    launch_debug_nodes = LaunchConfiguration('launch_debug_nodes')
    use_simulation = LaunchConfiguration('use_simulation')
    robot_environment = LaunchConfiguration('robot_environment')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'launch_debug_nodes',
            default_value='false',
            choices=['true', 'false'],
            description='Launch debug nodes like rviz, rqt tools'
        ),
        
        DeclareLaunchArgument(
            'use_simulation',
            default_value='false',
            choices=['true', 'false'],
            description='Launch with simulation environment'
        ),
        
        DeclareLaunchArgument(
            'robot_environment',
            default_value='indoor',
            choices=['indoor', 'outdoor', 'dynamic'],
            description='Environment configuration for the robot'
        ),
        
        # Main robot controller always launches
        Node(
            package='humanoid_controller',
            executable='main_controller',
            name='main_controller',
            parameters=[
                {'environment': robot_environment},
                {'use_sim_time': use_simulation}
            ],
            output='screen'
        ),
        
        # Conditional debug tools launch
        Node(
            package='rviz2',
            executable='rviz2', 
            name='rviz2_debug',
            arguments=['-d', 'config/humanoid_config.rviz'],
            condition=IfCondition(launch_debug_nodes),
            output='screen'
        ),
        
        # Conditional simulation-specific nodes
        GroupAction(
            condition=IfCondition(use_simulation),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'humanoid_robot',
                        '-file', 'models/humanoid/model.sdf',
                        '-x', '0', '-y', '0', '-z', '1.0'
                    ],
                    output='screen'
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='sim_robot_state_publisher',
                    parameters=[
                        {'use_sim_time': True}
                    ]
                )
            ]
        ),
        
        # Conditional real robot-specific nodes
        GroupAction(
            condition=UnlessCondition(use_simulation),
            actions=[
                Node(
                    package='imu_driver',
                    executable='imu_driver_node',
                    name='real_imu_driver',
                    parameters=[
                        {'port': '/dev/ttyUSB0'},
                        {'baud_rate': 115200}
                    ]
                ),
                Node(
                    package='lidar_driver',
                    executable='lidar_driver_node',
                    name='real_lidar_driver',
                    parameters=[
                        {'device_ip': '192.168.1.10'}
                    ]
                )
            ]
        ),
        
        # Environment-specific configurations
        Node(
            package='navigation2',
            executable='bt_navigator',
            name='nav2_bt_navigator',
            parameters=[
                {'use_sim_time': use_simulation},
                {'environment_config': [robot_environment, '_config.yaml']}
            ],
            condition=IfCondition(PythonExpression([robot_environment, " != 'dynamic'"]))
        ),
        
        # Log current configuration
        LogInfo(
            msg=["Robot configured for environment: ", robot_environment,
                 " with simulation mode: ", use_simulation]
        )
    ])