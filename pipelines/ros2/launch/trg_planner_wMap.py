import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _truthy(value):
    return value.lower() in ("1", "true", "yes", "on")


def _find_workspace_rviz(package_share_directory):
    for parent in Path(package_share_directory).parents:
        candidate = parent / "bringup" / "rviz" / "robocup_navigation.rviz"
        if candidate.is_file():
            return str(candidate)
    return ""


def _launch_rviz(context, *args, **kwargs):
    if not _truthy(LaunchConfiguration("rviz").perform(context)):
        return []

    package_share_directory = kwargs["package_share_directory"]
    explicit_config = LaunchConfiguration("rviz_config").perform(context)
    map_name = LaunchConfiguration("map").perform(context)

    if explicit_config:
        rviz_config_file = explicit_config
    else:
        map_config = os.path.join(package_share_directory, "rviz", f"{map_name}.rviz")
        if os.path.isfile(map_config):
            rviz_config_file = map_config
        else:
            rviz_config_file = _find_workspace_rviz(package_share_directory)
        if not rviz_config_file:
            rviz_config_file = os.path.join(package_share_directory, "rviz", "mountain.rviz")

    return [
        LogInfo(msg=["Using RViz config: ", rviz_config_file]),
        Node(package="rviz2",
             executable="rviz2",
             name="rvizualizer",
             arguments=["-d", rviz_config_file],
             output="screen"),
    ]


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
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Explicit RViz config file. Defaults to map RViz, then RoboCup workspace RViz.')

    # Get the path to the configuration files
    package_share_directory = get_package_share_directory('trg_planner_ros')

    # Get the path to the configuration files
    ros2_config_file = PathJoinSubstitution(
        [package_share_directory, 'config', LaunchConfiguration('params')])

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

    return LaunchDescription([
        rviz_arg,
        map_config_arg,
        params_arg,
        rviz_config_arg,
        ros2_node,
        OpaqueFunction(function=_launch_rviz,
                       kwargs={'package_share_directory': package_share_directory}),
    ])
