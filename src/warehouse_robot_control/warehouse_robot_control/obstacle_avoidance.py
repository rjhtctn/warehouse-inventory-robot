from dataclasses import fields
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from warehouse_robot_control.avoidance import Avoidance, Config, summarize_scan


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        defaults = Config()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                  for f in fields(Config)}
        self.logic = Avoidance(Config(**values))
        self.scan = None
        self.received = 0.0
        self.stamp = 0.0
        self.sequence = 0
        self.used_sequence = 0
        self.previous_time = 0.0
        self.previous_state = ''
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, '/avoidance/state', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.control, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_scan(self, msg):
        self.scan = summarize_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                   msg.range_min, msg.range_max)
        self.received = time.monotonic()
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.sequence += 1

    def control(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if now < self.previous_time:
            self.logic.reset()
            self.scan = None
        self.previous_time = now
        scan = self.scan
        timeout = self.logic.config.scan_timeout
        if (now <= 0 or time.monotonic()-self.received > timeout
                or not -0.05 <= now-self.stamp <= timeout):
            scan = None
        linear, angular = self.logic.step(scan, now, self.sequence != self.used_sequence)
        self.used_sequence = self.sequence
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        self.state_publisher.publish(String(data=self.logic.state))
        if self.logic.state != self.previous_state:
            self.get_logger().info(self.logic.state)
            self.previous_state = self.logic.state


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
