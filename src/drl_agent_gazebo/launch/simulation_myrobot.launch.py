#!/usr/bin/python3
"""
MyRobot差速机器人仿真启动文件
使用用户自定义的差速机器人进行DRL导航训练
采用模块化方式，引用gazebo_world.launch.py加载世界
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
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
        default_value="true",
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
    
    # ============================================================================
    # 设置 GAZEBO_MODEL_PATH（与 gazebo_world.launch.py 保持一致）
    # ============================================================================
    drl_agent_description_pkg = "drl_agent_description"
    velodyne_description_pkg = "velodyne_description"
    
    drl_agent_description_share = get_package_share_directory(drl_agent_description_pkg)
    velodyne_description_share = get_package_share_directory(velodyne_description_pkg)
    
    # 收集所有模型路径
    gazebo_resource_paths = [
        get_package_prefix(drl_agent_gazebo_pkg) + "/share",
        get_package_prefix(drl_agent_description_pkg) + "/share",
        get_package_prefix(myrobot_pkg) + "/share",
        os.path.join(drl_agent_description_share, "meshes"),
        os.path.join(drl_agent_description_share, "models"),
        os.path.join(drl_agent_gazebo_share, "models"),
        os.path.join(myrobot_share, "models"),
        get_package_prefix(velodyne_description_pkg) + "/share",
        os.path.join(velodyne_description_share, "meshes"),
    ]
    
    # 设置环境变量
    if "GAZEBO_MODEL_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["GAZEBO_MODEL_PATH"]:
                os.environ["GAZEBO_MODEL_PATH"] += ":" + resource_path
    else:
        os.environ["GAZEBO_MODEL_PATH"] = ":".join(gazebo_resource_paths)
    
    print("+" + "-" * 80 + "+")
    print("> GAZEBO MODELS PATH: ")
    print(str(os.environ["GAZEBO_MODEL_PATH"]))
    print("+" + "-" * 80 + "+")
    
    # ============================================================================
    # Launch配置
    # ============================================================================
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    world_name = LaunchConfiguration("world")
    
    # 世界文件路径（使用myrobot_sim_gazebo中的世界文件）
    world_path = PathJoinSubstitution([myrobot_share, "worlds", world_name])
    
    # 引用 gazebo_world.launch.py 加载世界
    gazebo_world_launch = PathJoinSubstitution(
        [drl_agent_gazebo_share, "launch", "gazebo_world.launch.py"]
    )
    
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_world_launch]),
        launch_arguments=[
            ("use_gazebo_gui", use_gazebo_gui),
            ("world_path", world_path),
        ],
    )
    
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
    
    # RViz（可选）- 使用与simulation_2d_lidar.launch.py相同的配置
    rviz_launch = PathJoinSubstitution(
        [drl_agent_gazebo_share, "launch", "rviz_myrobot.launch.py"]
    )
    
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[("use_sim_time", use_sim_time)],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    
    # 构建Launch描述
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo_world)
    ld.add_action(spawn_entity)
    ld.add_action(rviz2)
    
    return ld
