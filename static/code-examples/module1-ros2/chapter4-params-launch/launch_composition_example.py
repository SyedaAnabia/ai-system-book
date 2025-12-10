# launch_composition_example.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Create a container for composable nodes - more efficient than separate processes
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # Multi-threaded container
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[
                # Image processing node
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                    remappings=[
                        ('image', 'camera/image_raw'),
                        ('camera_info', 'camera/camera_info'),
                        ('image_rect', 'camera/image_rect')
                    ]
                ),
                
                # Point cloud processing node
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='pointcloud_xyzrgb_node',
                    remappings=[
                        ('rgb/image_rect_color', 'camera/image_rect'),
                        ('rgb/camera_info', 'camera/camera_info'),
                        ('depth_registered/image_rect', 'depth/image_rect'),
                        ('points', 'camera/points')
                    ]
                ),
                
                # Laser scan projection node
                ComposableNode(
                    package='laser_filters',
                    plugin='laser_filters::ScanMaskFilter',
                    name='scan_mask_filter',
                    remappings=[
                        ('scan', 'base_scan'),
                        ('scan_filtered', 'masked_base_scan')
                    ]
                )
            ],
            output='screen'
        ),

        # Second container for control nodes
        ComposableNodeContainer(
            name='control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[
                # Joint state broadcaster
                ComposableNode(
                    package='joint_state_broadcaster',
                    plugin='joint_state_broadcaster::JointStateBroadcaster',
                    name='joint_state_broadcaster'
                ),
                
                # Diff drive controller
                ComposableNode(
                    package='diff_drive_controller',
                    plugin='diff_drive_controller::DiffDriveController',
                    name='diff_drive_controller',
                    parameters=[
                        {'left_wheel_names': ['left_wheel']},
                        {'right_wheel_names': ['right_wheel']},
                        {'wheel_separation': 0.5},
                        {'wheel_radius': 0.1}
                    ]
                )
            ],
            output='screen'
        )
    ])