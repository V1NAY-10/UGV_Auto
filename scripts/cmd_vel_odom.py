#!/usr/bin/env python3
"""Fallback odometry for the real robot: dead reckoning from /cmd_vel.

The motors have no encoders, so the preferred odometry source is rf2o
laser odometry (see robot.launch.py). Use this node only if rf2o is not
available. It integrates the *commanded* velocity, so it drifts, but
slam_toolbox / AMCL correct the drift through the map->odom transform.

Publishes /odom and the odom -> base_link TF.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class CmdVelOdom(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        rate = self.declare_parameter('rate', 30.0).value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / rate, self.update)

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now

        # Same timeout as the ESP32 watchdog: no command -> robot stopped
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.v = 0.0
            self.w = 0.0

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        # Large covariance: this is open-loop and not to be trusted much
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = CmdVelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
