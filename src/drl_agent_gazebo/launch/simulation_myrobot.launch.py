#!/usr/bin/python3
"""
MyRobot差速机器人仿真启动文件
使用用户自定义的差速机器人进行DRL导航训练
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="使用仿真时间",
    ),
    DeclareLaunchArgument(
        "use_gazebo_gui",
        default_value="true",
        choices=["true", "false"],
        description="启动Gazebo GUI",
    ),
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        choices=["true", "false"],
        description="启动RViz",
    ),
    DeclareLaunchArgument(
        "world",
        default_value="td7_static.world",
        description="Gazebo世界文件名称",
    ),
]


def generate_launch_description():
    # 获取包路径
    myrobot_pkg = "myrobot_sim_gazebo"
    drl_agent_gazebo_pkg = "drl_agent_gazebo"
    
    myrobot_share = get_package_share_directory(myrobot_pkg)
    drl_agent_gazebo_share = get_package_share_directory(drl_agent_gazebo_pkg)
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    
    # Launch配置
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    world_name = LaunchConfiguration("world")
    
    # 世界文件路径（使用myrobot_sim_gazebo中的世界文件）
    world_path = PathJoinSubstitution([myrobot_share, "worlds", world_name])
    
    # 机器人URDF处理
    xacro_file = os.path.join(myrobot_share, "xacro", "myrobot_lidar.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}
    
    # Robot State Publisher节点
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    
    # Gazebo服务器
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )
    
    # Gazebo客户端
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(use_gazebo_gui),
    )
    
    # 生成机器人实体
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "myrobot_diff_bot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-Y", "0.0",
        ],
        output="screen",
    )
    
    # RViz（可选）
    rviz_config = os.path.join(drl_agent_gazebo_share, "rviz", "rviz.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    
    # 构建Launch描述
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_entity)
    ld.add_action(rviz_node)
    
    return ld
