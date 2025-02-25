import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',  # Default is not to launch RViz
        description='Set to "true" to launch RViz'
    )

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("em_sim_bringup")
    pkg_project_gazebo = get_package_share_directory("em_sim_gazebo")
    pkg_project_description = get_package_share_directory("em_sim_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_em_vehicle_control = get_package_share_directory("em_vehicle_control")

    # read config
    yaml_file = os.path.join(pkg_project_bringup, "config", "bringup.yaml")
    with open(yaml_file, "r") as f:
        config = yaml.safe_load(f)

    # Load XACRO file
    xacro_file = os.path.join(pkg_project_description, "urdf", "edison_simple.xacro")

    # Launch simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [pkg_project_gazebo, "worlds", "track_sim.sdf"]
            )
        }.items(),
    )
    robot_nodes = []
    for i in range(config["num_robots"]):
        robot_name = f"robot_{i}"

        # Generate the URDF file
        urdf_file = os.path.join(
            pkg_project_description, "urdf", f"edison_simple_{robot_name}.urdf"
        )
        # Command to convert Xacro to URDF
        xacro_command = [
            'xacro',
            xacro_file,
            f'robotNamespace:={robot_name}',
            '>', urdf_file
        ]

        print(f"Generated Xacro Command for {robot_name}: {' '.join(xacro_command)}")

        # ExecuteProcess to run xacro
        generate_urdf = ExecuteProcess(
            cmd=['/bin/bash', '-c', ' '.join(xacro_command)],
            output='screen'
        )
        robot_nodes.append(generate_urdf)

        # Spawn the robot using the `spawn_entity` service from ros_gz_interfaces
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file",
                urdf_file,
                "-name",
                robot_name,  # Name of the entity in Gazebo
                "-x",
                str(config["poses"][i][0]),
                "-y",
                str(config["poses"][i][1]),
                "-z",
                "0.1",  # Initial position of the robot
                "-Y",
                str(config["poses"][i][2]),
            ],
            output="screen",
        )
        # robot_nodes.append(spawn_robot)

        spawn_robot_after_xacro = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=generate_urdf, on_exit=[spawn_robot]
            )
        )
        robot_nodes.append(spawn_robot_after_xacro)

        # Start ros2 to gazebo bridge for topic transfer
        tf_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge_tf",
            namespace=robot_name,
            arguments=[
                f"/{robot_name}/pose@nav_msgs/msg/Odometry@gz.msgs.Odometry"
            ],
            output="screen",
        )
        robot_nodes.append(tf_bridge)

        # Convert pose message from tf_bridge to tf tree
        odom_base_link_tf_broadcaster = Node(
            package="em_sim_bringup",
            executable="gazebo_tf_broadcaster",
            name="gazebo_tf_broadcaster",
            namespace=robot_name,
            parameters=[{"robot_name": robot_name}],
            output="screen",
        )
        robot_nodes.append(odom_base_link_tf_broadcaster)

        # Static tf broadcaster for world to odom frame of each robot
        # static_tf = Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="static_transform_publisher",
        #     namespace=robot_name,
        #     arguments=[
        #         str(config["poses"][i][0]),  # x
        #         str(config["poses"][i][1]),  # y
        #         "0",  # z
        #         str(config["poses"][i][2]),  # y
        #         "0",  # p
        #         "0",  # r
        #         "world",  # parent
        #         f"{robot_name}/odom",  # child
        #     ],
        # )
        # robot_nodes.append(static_tf)

        # Start ros2 to gazebo bridge for twist message transfer
        cmd_vel_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge_cmd_vel",
            namespace=robot_name,
            arguments=[
                f"/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"
            ],
            output="screen",
        )
        robot_nodes.append(cmd_vel_bridge)

        # start tracking node
        tracker = Node(
            package="em_vehicle_control",
            executable='tracker',
            name='pathtracker',
            namespace=robot_name,
            parameters=[{
                "robot_name":robot_name
            }],
            output='screen',
            prefix=["taskset -c ", str(i % os.cpu_count())]
        )
        robot_nodes.append(tracker)

    # Publishes gazebo map as an occupancy grid for rviz
    pub_road_network = Node(
            package='em_sim_gazebo',
            executable='sdf_to_occupancy_grid',
            name='sdf_to_occupancy_grid_node',
            output='screen'
        )
    
    # start path planning node
    path_planner = Node(
        package='em_vehicle_control',
        executable='planner',
        name='path_planner_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_project_bringup, 'rviz', 'config.rviz2'])],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    ld = LaunchDescription(
        [
            declare_rviz_arg,
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=PathJoinSubstitution([pkg_project_gazebo, "worlds"]),
            ),
            gz_sim,
            *robot_nodes,
            pub_road_network,
            path_planner,
            rviz_node
        ]
    )

    return ld
