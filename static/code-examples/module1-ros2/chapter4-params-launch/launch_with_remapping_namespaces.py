# launch_with_remapping_namespaces.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Declare launch arguments
    robot_namespace = LaunchConfiguration('robot_namespace')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='humanoid_robot',
            description='Namespace for the robot nodes'
        ),
        
        # Use the robot namespace for all nodes
        PushRosNamespace(robot_namespace),
        
        # Humanoid controller node with remapping
        Node(
            package='humanoid_controller',
            executable='humanoid_controller_node',
            name='controller',
            parameters=[
                {'use_sim_time': False},
                {'robot_model': 'atlas_v5'}
            ],
            remappings=[
                ('/joint_states', 'input_joint_states'),
                ('/cmd_vel', 'navigation/cmd_vel'),
                ('/imu/data', 'sensors/imu/data')
            ],
            output='screen'
        ),
        
        # Sensor processor node with remapping
        Node(
            package='sensor_processor',
            executable='lidar_processor_node',
            name='lidar_processor',
            parameters=[
                {'use_sim_time': False},
                {'scan_topic': 'scan'},
                {'range_threshold': 10.0}
            ],
            remappings=[
                ('/scan', 'sensors/lidar_scan'),
                ('/processed_scan', 'perception/processed_scan')
            ],
            output='screen'
        ),
        
        # Vision processor node with remapping
        Node(
            package='vision_processor',
            executable='camera_processor_node',
            name='camera_processor',
            parameters=[
                {'use_sim_time': False},
                {'image_topic': 'image_raw'},
                {'enable_rectification': True}
            ],
            remappings=[
                ('/image_raw', 'sensors/camera/image_raw'),
                ('/image_rect', 'vision/image_rect')
            ],
            output='screen'
        )
    ])