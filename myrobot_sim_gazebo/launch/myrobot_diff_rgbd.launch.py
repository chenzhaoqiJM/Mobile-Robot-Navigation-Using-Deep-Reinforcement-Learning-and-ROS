import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    package_name='myrobot_sim_gazebo' #<--- CHANGE ME
    world_file_path = 'worlds/turtlebot3_world.world'

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)

    # robot state publisher
    xacro_file = os.path.join(pkg_path,'xacro','myrobot_rgbd2.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}

    myrobot_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Pose where we want to spawn the robot
    spawn_x_val = '-2.0'
    spawn_y_val = '-0.5'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),

                launch_arguments={'world': world_path}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'myrobot_diff_bot',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        myrobot_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
