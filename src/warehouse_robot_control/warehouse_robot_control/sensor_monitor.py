from math import atan2, asin
import time
import zlib

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Imu, LaserScan

from warehouse_robot_control.avoidance import summarize_scan


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.sensor = self.declare_parameter('sensor', 'lidar').value
        types = {'lidar': (LaserScan, '/scan'), 'camera': (Image, '/camera/image_raw'),
                 'imu': (Imu, '/imu/data')}
        if self.sensor not in types:
            raise ValueError('sensor must be lidar, camera, or imu')
        self.count = 0
        self.previous_count = 0
        self.last_received = 0.0
        self.description = 'waiting for data'
        msg_type, topic = types[self.sensor]
        self.create_subscription(msg_type, topic, self.receive, qos_profile_sensor_data)
        self.create_timer(2.0, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        self.count += 1
        self.last_received = time.monotonic()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        prefix = f'frame={msg.header.frame_id} stamp={stamp:.3f}'
        if self.sensor == 'lidar':
            scan = summarize_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                  msg.range_min, msg.range_max)
            self.description = prefix + (f' front={scan.front:.2f}m nearest={scan.nearest:.2f}m'
                                         if scan else ' unusable scan')
        elif self.sensor == 'camera':
            valid = len(msg.data) == msg.step*msg.height and msg.width > 0 and msg.height > 0
            self.description = (f'{prefix} {msg.width}x{msg.height} {msg.encoding}'
                                f' valid={valid} crc32={zlib.crc32(msg.data):08x}')
        else:
            q = msg.orientation
            roll = atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
            yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            a, w = msg.linear_acceleration, msg.angular_velocity
            self.description = (f'{prefix} rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})'
                                f' gyro=({w.x:.3f},{w.y:.3f},{w.z:.3f})'
                                f' accel=({a.x:.3f},{a.y:.3f},{a.z:.3f})')

    def report(self):
        rate = (self.count-self.previous_count)/2.0
        self.previous_count = self.count
        if time.monotonic()-self.last_received > 2.0:
            self.get_logger().warning(f'{self.sensor}: no recent data')
        else:
            self.get_logger().info(f'{self.sensor}: {rate:.1f}Hz {self.description}')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
