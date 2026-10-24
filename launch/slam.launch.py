"""SLAM: build a 2D occupancy-grid map from /scan with slam_toolbox.

Run on top of sim.launch.py (use_sim_time:=true) or robot.launch.py
(use_sim_time:=false, the default). Drive around with teleop, then save:
  ros2 run nav2_map_server map_saver_cli -f ~/ugv_maps/my_map
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('ugv_auto')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_path, 'config', 'slam_params.yaml'),
            description='slam_toolbox parameters file'),

        slam_toolbox,
    ])
