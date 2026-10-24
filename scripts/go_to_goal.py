#!/usr/bin/env python3
"""Send the UGV from point A to point B using Nav2.

Point A is where the robot currently is. With a saved map + AMCL, tell
AMCL where A is with --start (or use "2D Pose Estimate" in RViz).
Point B is the goal, given in map coordinates (metres).

Examples:
  # Navigating on a saved map (AMCL): robot starts at origin, go to (4.0, -0.5)
  ros2 run ugv_auto go_to_goal.py --start 0 0 0 --goal 4.0 -0.5 0

  # Navigating while mapping (slam:=true): no initial pose needed
  ros2 run ugv_auto go_to_goal.py --goal 4.0 -0.5 0

  # Several goals in a row: A -> B -> C
  ros2 run ugv_auto go_to_goal.py --goal 4.0 -0.5 0 --goal 0 0 3.14
"""
import argparse
import math
import sys

import rclpy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # Zero stamp = "latest transform", works with both sim time and wall time
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    parser = argparse.ArgumentParser(description='Navigate the UGV from A to B.')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='Initial pose (point A) for AMCL. Omit when using SLAM.')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        action='append', required=True,
                        help='Goal pose (point B). Repeat for multiple goals.')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    navigator = BasicNavigator()

    if args.start is not None:
        navigator.setInitialPose(make_pose(*args.start))
        # Waits for AMCL and bt_navigator to be active
        navigator.waitUntilNav2Active()
    # Without --start (SLAM mode) goToPose itself waits for the action server

    for i, (x, y, yaw) in enumerate(args.goal):
        navigator.get_logger().info(f'Goal {i + 1}/{len(args.goal)}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        navigator.goToPose(make_pose(x, y, yaw))

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                navigator.get_logger().info(
                    f'Distance remaining: {feedback.distance_remaining:.2f} m',
                    throttle_duration_sec=2.0)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('Goal reached.')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn('Goal was canceled.')
            break
        else:
            navigator.get_logger().error('Goal failed.')
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
