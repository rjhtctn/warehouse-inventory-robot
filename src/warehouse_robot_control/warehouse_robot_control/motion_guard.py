from math import isfinite
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class MotionGuard(Node):
    def __init__(self):
        super().__init__('motion_guard')
        self.autonomous = self.declare_parameter('autonomous', False).value
        self.controller = self.declare_parameter(
            'controller', 'avoidance' if self.autonomous else 'manual').value
        if self.controller not in ('manual', 'avoidance', 'inspection', 'patrol'):
            raise ValueError('Unknown motion controller')
        self.timeout = self.declare_parameter('command_timeout', 0.35).value
        self.max_linear = self.declare_parameter('max_linear_speed', 0.30).value
        self.max_angular = self.declare_parameter('max_angular_speed', 0.80).value
        if any(not isfinite(v) or v <= 0 for v in (self.timeout, self.max_linear, self.max_angular)):
            raise ValueError('Motion limits and timeout must be positive')
        self.allowed = set()
        self.command = Twist()
        self.received = 0.0
        self.source = None
        self.publisher = self.create_publisher(Twist, '/drive/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.update_sources, clock=steady)
        self.create_timer(0.05, self.publish, clock=steady)

    def update_sources(self):
        endpoints = self.get_publishers_info_by_topic('/cmd_vel')
        if self.controller != 'manual':
            name = {'avoidance': 'obstacle_avoidance', 'inspection': 'inventory_mission',
                    'patrol': 'inventory_patrol'}[self.controller]
            endpoints = [e for e in endpoints if e.node_name == name
                         and e.node_namespace == '/']
        else:
            endpoints = [e for e in endpoints if e.node_name not in
                         ('obstacle_avoidance', 'inventory_mission', 'inventory_patrol')]
        # Refuse ambiguous command ownership instead of interleaving publishers.
        self.allowed = {bytes(e.endpoint_gid) for e in endpoints} if len(endpoints) == 1 else set()

    def on_command(self, msg, info):
        gid = info['publisher_gid']
        if gid is None:
            return
        source = bytes(gid['data'])
        if source not in self.allowed:
            return
        if not all(isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.command = Twist()
        else:
            self.command = Twist()
            self.command.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
            self.command.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self.source = source
        self.received = time.monotonic()

    def publish(self):
        fresh = time.monotonic()-self.received <= self.timeout and self.source in self.allowed
        self.publisher.publish(self.command if fresh else Twist())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionGuard()
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
