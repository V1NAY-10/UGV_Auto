#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.last_time = time.time()

    def cmd_callback(self, msg):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        v = msg.linear.x
        w = msg.angular.z

        self.x  += v * math.cos(self.yaw) * dt
        self.y  += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.yaw/2)

        self.pub.publish(odom)

def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
