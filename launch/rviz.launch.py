"""RViz with the Nav2 view (map, scan, costmaps, path, 2D Pose Estimate, Nav2 Goal).

  ros2 launch ugv_auto rviz.launch.py use_sim_time:=true   # with Gazebo
  ros2 launch ugv_auto rviz.launch.py                      # with the real robot (run on laptop)
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config,
            description='RViz config file'),

        rviz,
    ])
