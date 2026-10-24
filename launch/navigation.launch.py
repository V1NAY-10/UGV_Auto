"""Nav2 autonomous navigation (point A -> point B).

Two modes:
  * On a saved map (AMCL localization) - the normal case after mapping:
      ros2 launch ugv_auto navigation.launch.py map:=$HOME/ugv_maps/my_map.yaml
  * While mapping (slam_toolbox provides map + localization):
      ros2 launch ugv_auto navigation.launch.py slam:=true

Add use_sim_time:=true when running against Gazebo.
Then set point A with "2D Pose Estimate" (AMCL mode only) and point B with
"Nav2 Goal" in RViz, or use: ros2 run ugv_auto go_to_goal.py --goal X Y YAW
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_path = get_package_share_directory('ugv_auto')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')

    # Saved map + AMCL (map -> odom TF from AMCL)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'localization_launch.py')),
        condition=UnlessCondition(slam),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    # Live SLAM (map -> odom TF from slam_toolbox)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'slam.launch.py')),
        condition=IfCondition(slam),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Planner, controller, behaviours, BT navigator, velocity smoother
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_path, 'maps', 'map.yaml'),
            description='Saved map yaml (ignored when slam:=true)'),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Navigate while mapping with slam_toolbox instead of AMCL'),

        localization,
        slam_launch,
        navigation,
    ])
