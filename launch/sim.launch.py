"""Gazebo simulation of the UGV: world + robot + robot_state_publisher.

  ros2 launch ugv_auto sim.launch.py
  ros2 launch ugv_auto sim.launch.py world:=/path/to/other.world
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('ugv_auto')
    world = LaunchConfiguration('world')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ugv_auto',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', '0.08'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_path, 'worlds', 'obstacles.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y (m)'),

        rsp,
        gazebo,
        spawn_entity,
    ])
