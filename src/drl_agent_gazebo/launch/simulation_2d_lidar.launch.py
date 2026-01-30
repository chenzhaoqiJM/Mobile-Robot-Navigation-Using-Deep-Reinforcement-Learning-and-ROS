#!/usr/bin/python3
"""
2D激光雷达版本的仿真launch文件
使用Hokuyo 2D激光雷达代替Velodyne 3D雷达进行DRL训练
"""

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="use_sim_time",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Start rviz",
    ),
    DeclareLaunchArgument(
        "use_gazebo_gui",
        default_value="true",
        choices=["true", "false"],
        description="Start gzclient",
    ),
    DeclareLaunchArgument(
        "slam",
        default_value="false",
        choices=["true", "false"],
        description="Whether or not to launch SLAMToolBox",
    ),
]


def generate_launch_description():
    # Get the agent_description share directory
    drl_agent_gazebo_pkg = "drl_agent_gazebo"
    drl_agent_description_pkg = "drl_agent_description"
    drl_agent_gazebo_share = get_package_share_directory(drl_agent_gazebo_pkg)
    drl_agent_description_share = get_package_share_directory(drl_agent_description_pkg)

    # Paths - 使用2D激光雷达版本的launch文件
    agent_description_launch = PathJoinSubstitution(
        [drl_agent_description_share, "launch", "agent_description_2d_lidar.launch.py"]
    )
    agent_gazebo_world_launch = PathJoinSubstitution(
        [drl_agent_gazebo_share, "launch", "gazebo_world.launch.py"]
    )
    agent_spawn_launch = PathJoinSubstitution(
        [drl_agent_gazebo_share, "launch", "spawn_agent.launch.py"]
    )
    rviz_launch = PathJoinSubstitution(
        [drl_agent_gazebo_share, "launch", "rviz.launch.py"]
    )
    slam_launch = PathJoinSubstitution(
        [drl_agent_gazebo_share, "launch", "slam.launch.py"]
    )

    # Launch configurations
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")

    # agent description (使用2D激光雷达版本)
    agent_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_description_launch]),
        launch_arguments=[("namespace", namespace), ("use_sim_time", use_sim_time)],
    )

    # Gazebo world
    agent_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_gazebo_world_launch]),
        launch_arguments=[
            ("use_gazebo_gui", use_gazebo_gui),
        ],
    )

    # Agent spawn
    agent_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [agent_spawn_launch]
        )  # TODO: add location as argument
    )

    # Rviz
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[("namespace", namespace), ("use_sim_time", use_sim_time)],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[("namespace", namespace), ("use_sim_time", use_sim_time)],
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(agent_description)
    ld.add_action(agent_gazebo_world)
    ld.add_action(agent_spawn)
    ld.add_action(rviz2)
    ld.add_action(slam)

    return ld
