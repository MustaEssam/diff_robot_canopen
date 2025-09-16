"""
Launch file for starting the robot control nodes.

This launch file is responsible for the following:
- Loading the robot description (URDF) from a xacro file.
- Loading the controller configuration from a YAML file.
- Starting the controller manager node.
- Spawning the joint state broadcaster and diff drive controller.
- Starting the robot state publisher node.
- Starting RViz for visualization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generates the launch description for the robot control system.

    Returns:
        LaunchDescription: The launch description object.
    """
    # Path to the robot's URDF file (xacro)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("diff_robot_canopen"),
                    "urdf/robot_controller",
                    "robot_controller.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Path to the ros2_controllers configuration file
    robot_control_config = PathJoinSubstitution(
        [FindPackageShare("diff_robot_canopen"), "config/robot_control", "ros2_controllers.yaml"]
    )

    # Path to the RViz configuration file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diff_robot_canopen"), "rviz", "view_robot.rviz"]
    )

    # Node for the controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
        output="screen",
    )

    # Node for spawning the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Node for spawning the diff drive controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Node for the robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Node for RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file],
    )

    # List of nodes to start
    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz_node
    ]

    return LaunchDescription(nodes_to_start)