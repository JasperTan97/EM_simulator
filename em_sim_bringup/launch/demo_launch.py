from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # Setup project paths
    pkg_project_gazebo = get_package_share_directory("em_sim_gazebo")

    robot_name = "robot_0"

    # start tracking node
    tracker = Node(
        package="em_vehicle_control",
        executable="tracker_demo",
        name="pathtracker",
        parameters=[{"robot_name": robot_name}],
        output="screen",
    )

    # start path planning node
    path_planner = Node(
        package="em_vehicle_control",
        executable="planner_demo",
        name="path_planner_node",
        output="screen",
    )

    ld = LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=PathJoinSubstitution([pkg_project_gazebo, "worlds"]),
            ),
            tracker,
            path_planner,
        ]
    )

    return ld
