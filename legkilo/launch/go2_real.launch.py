import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('legkilo')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'go2_real.yaml'),
        description='Path to the YAML config file'
    )
    
    # Paths to files
    # urdf_file = os.path.join(pkg_share, 'robot_urdf', 'go2_description', 'urdf', 'go2.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'legkilo_ros2.rviz')
    
    # Read URDF content
    # try:
    #     with open(urdf_file, 'r') as f:
    #         robot_description = f.read()
    # except FileNotFoundError:
        # robot_description = ''
    
    # Legkilo node
    legkilo_node = Node(
        package='legkilo',
        executable='legkilo_node',
        name='legkilo_node',
        output='screen',
        arguments=['--config_file', LaunchConfiguration('config_file')],
    )
    
    # # Robot state publisher node
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[{
    #         'robot_description': robot_description,
    #         'publish_frequency': 1000.0,
    #     }],
    # )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    return LaunchDescription([
        config_file_arg,
        legkilo_node,
        # robot_state_publisher_node,
        rviz_node,
    ])