import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument('rviz',
                                     default_value='false',
                                     description='Whether to launch RViz')
    map_config_arg = DeclareLaunchArgument(
        'map',
        default_value='mountain',
        description='Configuration file to use')
    params_arg = DeclareLaunchArgument(
        'params',
        default_value='ros2_params.yaml',
        description='ROS parameter YAML file in the package config directory')

    # Get the path to the configuration files
    package_share_directory = get_package_share_directory('trg_planner_ros')

    # Get the path to the configuration files
    ros2_config_file = PathJoinSubstitution(
        [package_share_directory, 'config', LaunchConfiguration('params')])
    rviz_config_file = PathJoinSubstitution(
        [package_share_directory, 'rviz', [LaunchConfiguration('map'), '.rviz']])

    # Load rosparam (YAML file)
    ros2_node = Node(package='trg_planner_ros',
                     executable='trg_ros2_node',
                     name='trg_ros2_node',
                     parameters=[
                         ros2_config_file, {
                             'mapConfig': LaunchConfiguration('map')
                         }
                     ],
                     output='screen')

    # Define the RViz node
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rvizualizer',
                     arguments=['-d', rviz_config_file],
                     condition=IfCondition(LaunchConfiguration('rviz')),
                     output='screen')

    return LaunchDescription([rviz_arg, map_config_arg, params_arg, ros2_node, rviz_node])
