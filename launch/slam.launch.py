from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    # Path to your robot description package
    pkg_share = os.path.join(os.environ['HOME'], 'ros2_ws', 'src', 'my_bot')

    # Use xacro to generate URDF
    robot_description = Command([
        'xacro ',
        pkg_share + '/description/robot.urdf.xacro'
    ])

    return LaunchDescription([

        # -------------------------
        # Robot State Publisher
        # -------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # -------------------------
        # SLAM Toolbox
        # -------------------------
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'slam_params_file': pkg_share + '/config/slam_params.yaml'
            }]
        ),

    ])
