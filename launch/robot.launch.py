"""Real robot bringup - run this on the Raspberry Pi 4B.

Starts: robot_state_publisher, RPLIDAR A1 driver, odometry, ESP32 motor bridge.
Then run slam.launch.py (mapping) or navigation.launch.py (A -> B) on top.

  ros2 launch ugv_auto robot.launch.py
  ros2 launch ugv_auto robot.launch.py odom:=cmd_vel lidar_port:=/dev/ttyUSB0
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('ugv_auto')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # No wheel encoders -> publish zero wheel angles so the wheel TFs exist
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # ----- RPLIDAR A1 (Slamtec rplidar_ros, ros2 branch) -----
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,  # A1
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # ----- Odometry (no wheel encoders) -----
    # rf2o estimates odom -> base_link by matching consecutive laser scans
    rf2o = Node(
        condition=LaunchConfigurationEquals('odom', 'rf2o'),
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }]
    )

    # Fallback: integrate the commanded velocity
    cmd_vel_odom = Node(
        condition=LaunchConfigurationEquals('odom', 'cmd_vel'),
        package='ugv_auto',
        executable='cmd_vel_odom.py',
        output='screen'
    )

    # ----- Motors: /cmd_vel -> GPIO UART -> ESP32 -> 2x L298N -----
    esp32_bridge = Node(
        package='ugv_auto',
        executable='esp32_bridge.py',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'esp32_bridge.yaml')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ugv_lidar',
            description='RPLIDAR serial port (udev symlink, or /dev/ttyUSB0)'),
        DeclareLaunchArgument(
            'odom', default_value='rf2o',
            description='Odometry source: rf2o (laser odometry) or cmd_vel (dead reckoning)'),

        rsp,
        joint_state_publisher,
        rplidar,
        rf2o,
        cmd_vel_odom,
        esp32_bridge,
    ])
