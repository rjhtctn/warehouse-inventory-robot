from collections import deque
from array import array
from math import atan2, cos, hypot, isfinite, pi, sin
import time

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from warehouse_robot_interfaces.msg import InspectionResult

from warehouse_robot_control.avoidance import summarize_scan
from warehouse_robot_control.inspection_geometry import camera_lateral, fit_shelf, sector_clearance, synchronized_image, wrap, yaw_of
from warehouse_robot_control.inventory_vision import annotate, detect_boxes


def clamp(value, limit):
    return max(-limit, min(limit, value))


class InventoryMission(Node):
    terminal = ('COMPLETE', 'BLOCKED', 'FAILED')
    navigation_states = ('NAVIGATE',)
    viewing = ('ACQUIRE_VIEW', 'APPROACH', 'BACK_OFF', 'ALIGN', 'SETTLE', 'INSPECT')

    def __init__(self, name='inventory_mission', topic_prefix='/inspection'):
        super().__init__(name)
        defaults = dict(station_x=-1.45, station_front_y=1.64, shelf_heading=pi/2,
                        inspection_distance=2.4, linear_speed=.18, approach_speed=.07,
                        angular_speed=.4, sensor_timeout=.5, camera_timeout=.8,
                        startup_timeout=12., failure_timeout=2., mission_timeout=100.,
                        stable_seconds=1.2, stable_frames=8, initial_x=-2.8, initial_y=0.)
        self.config = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        c = self.config
        if (not all(isfinite(v) for v in c.values()) or c['inspection_distance'] < 1.5
                or not 0 < c['linear_speed'] <= .2 or not 0 < c['approach_speed'] <= .1
                or not 0 < c['angular_speed'] <= .5 or c['stable_frames'] < 3
                or any(c[k] <= 0 for k in ('sensor_timeout', 'camera_timeout', 'startup_timeout',
                                          'failure_timeout', 'mission_timeout', 'stable_seconds'))):
            raise ValueError('Invalid inspection parameters')
        cv2.setNumThreads(1)
        self.latest = {}
        self.image_history = deque(maxlen=3)
        self.state = 'WAIT_FOR_SENSORS'
        self.detail = ''
        self.since = self.started = time.monotonic()
        self.last_sim_time = 0.
        self.position = [c['initial_x'], c['initial_y']]
        self.last_odom = None
        self.heading = None
        self.scan_summary = None
        self.line = None
        self.lines = deque(maxlen=30)
        self.view_lateral = None
        self.view_position = None
        self.observation = None
        self.view_received = 0.
        self.processed_stamp = None
        self.mounts = None
        self.waypoint = 0
        self.backoffs = 0
        self.stable_since = None
        self.stable_count = 0
        self.last_counts = None
        self.last_center = None
        self.result_sent = False
        self.final_errors = (float('nan'),)*3
        self.interrupted_state = None
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.states = self.create_publisher(String, topic_prefix+'/state', retained)
        self.results = self.create_publisher(InspectionResult, topic_prefix+'/result', retained)
        self.images = self.create_publisher(Image, topic_prefix+'/image', retained)
        self.diagnostics = self.create_publisher(DiagnosticArray, topic_prefix+'/diagnostics', 5)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self, static_only=True)
        for name, typ, topic in [('scan', LaserScan, '/scan'), ('imu', Imu, '/imu/data'),
                                 ('odom', Odometry, '/odom'), ('joints', JointState, '/joint_states'),
                                 ('image', Image, '/camera/image_raw'),
                                 ('info', CameraInfo, '/camera/camera_info')]:
            self.create_subscription(typ, topic, lambda m, k=name: self.receive(k, m),
                                     qos_profile_sensor_data)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.control, clock=steady)
        self.create_timer(.5, self.process_image, clock=steady)
        self.create_timer(1., self.publish_status, clock=steady)
        self.get_logger().info(self.state)

    def receive(self, name, msg):
        if name == 'imu':
            try:
                heading = yaw_of(msg.orientation)
            except ValueError:
                return
            if not all(isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y,
                                            msg.angular_velocity.z, msg.linear_acceleration.z)):
                return
            self.heading = heading
        elif name == 'odom':
            p = msg.pose.pose.position
            if not all(isfinite(v) for v in (p.x, p.y, msg.twist.twist.linear.x,
                                            msg.twist.twist.angular.z)):
                return
            if self.last_odom is not None and self.heading is not None:
                x, y = self.last_odom
                distance = hypot(p.x-x, p.y-y)
                if distance > .10:
                    self.transition('FAILED', 'Odometry discontinuity')
                    return
                if msg.twist.twist.linear.x < 0:
                    distance = -distance
                self.position[0] += distance*cos(self.heading)
                self.position[1] += distance*sin(self.heading)
            self.last_odom = p.x, p.y
        elif name == 'scan':
            self.scan_summary = summarize_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                               msg.range_min, msg.range_max)
            if self.scan_summary is None:
                return
            if self.heading is not None and self.state in self.viewing:
                self.line = fit_shelf(msg.ranges, msg.angle_min, msg.angle_increment,
                                      self.heading, self.config['shelf_heading'],
                                      self.config['inspection_distance'])
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self.lines.append((stamp, self.line))
        elif name == 'joints':
            if (set(msg.name) != {'left_wheel_joint', 'right_wheel_joint'}
                    or len(msg.velocity) != 2 or not all(isfinite(v) for v in msg.velocity)):
                return
        elif name == 'image':
            if (msg.encoding != 'rgb8' or msg.width != 480 or msg.height != 360
                    or msg.step < msg.width*3 or len(msg.data) != msg.step*msg.height
                    or msg.header.frame_id != 'camera_optical_frame'):
                return
        elif name == 'info':
            if (msg.width != 480 or msg.height != 360 or not all(isfinite(v) for v in msg.k)
                    or msg.k[0] <= 0 or msg.k[4] <= 0
                    or msg.header.frame_id != 'camera_optical_frame'):
                return
        received = time.monotonic()
        self.latest[name] = received, msg
        if name == 'image':
            self.image_history.append((received, msg))

    def transition(self, state, detail=''):
        if self.state in self.terminal or state == self.state:
            return
        self.state, self.detail, self.since = state, detail, time.monotonic()
        self.stable_since = None
        if state not in ('COMPLETE', 'RECORD'):
            self.stable_count = 0
        self.states.publish(String(data=state))
        self.get_logger().info(state+(': '+detail if detail else ''))
        if state in self.terminal:
            self.command.publish(Twist())
            self.publish_result(state == 'COMPLETE')

    def stale_sensors(self):
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds*1e-9
        stale = []
        for key in ('scan', 'imu', 'odom', 'joints', 'image', 'info'):
            timeout = self.config['camera_timeout'] if key in ('image', 'info') else self.config['sensor_timeout']
            if key not in self.latest:
                stale.append(key)
                continue
            received, msg = self.latest[key]
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if wall-received > timeout or not -.05 <= now-stamp <= timeout:
                stale.append(key)
        if self.scan_summary is None and 'scan' not in stale:
            stale.append('scan')
        return stale

    def stationary(self):
        odom = self.latest['odom'][1].twist.twist
        imu = self.latest['imu'][1].angular_velocity
        return (abs(odom.linear.x) < .008 and abs(odom.angular.z) < .02
                and max(abs(imu.x), abs(imu.y), abs(imu.z)) < .02
                and max(abs(v) for v in self.latest['joints'][1].velocity) < .15)

    def load_mounts(self):
        try:
            self.mounts = tuple(self.buffer.lookup_transform('base_footprint', name,
                                rclpy.time.Time()).transform.translation.x
                                for name in ('camera_link', 'lidar_link'))
        except TransformException:
            return False
        return True

    def errors(self):
        if (self.line is None or self.observation is None or not self.observation.valid
                or self.view_lateral is None):
            return None
        heading = self.line.heading_error
        dx, dy = (self.position[i]-self.view_position[i] for i in range(2))
        normal = self.config['shelf_heading']
        lateral = self.view_lateral-(-sin(normal)*dx+cos(normal)*dy)
        return heading, self.line.distance-self.config['inspection_distance'], lateral

    def safe_command(self, linear=0., angular=0.):
        if self.state in self.terminal:
            linear = angular = 0.
        elif abs(linear)+abs(angular) > 0:
            scan = self.latest['scan'][1]
            nearest_limit = .38 if abs(angular) > .02 else .36
            direction = 0. if linear >= 0 else pi
            if (self.scan_summary.nearest < nearest_limit
                    or (abs(linear) > 0 and sector_clearance(scan, direction) < .50)):
                self.transition('BLOCKED', 'Obstacle violates route clearance')
                linear = angular = 0.
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.command.publish(msg)

    def control(self):
        if self.state in self.terminal:
            self.safe_command()
            return
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds*1e-9
        if now < self.last_sim_time:
            self.transition('FAILED', 'Simulation clock moved backwards')
            return
        self.last_sim_time = now
        stale = self.stale_sensors()
        if self.state == 'WAIT_FOR_SENSORS':
            self.safe_command()
            if not stale and now > 0 and self.load_mounts():
                self.transition('NAVIGATE')
            elif wall-self.since > self.config['startup_timeout']:
                self.transition('FAILED', 'Startup sensor/TF timeout: '+','.join(stale))
            return
        if stale:
            if self.state != 'SENSOR_TIMEOUT':
                self.interrupted_state = self.state
                self.transition('SENSOR_TIMEOUT', ','.join(stale))
            self.safe_command()
            if wall-self.since > self.config['failure_timeout']:
                self.transition('FAILED', 'Sensor timeout: '+','.join(stale))
            return
        if self.state == 'SENSOR_TIMEOUT':
            self.transition(self.interrupted_state)
        if wall-self.started > self.config['mission_timeout']:
            self.transition('FAILED', 'Mission progress timeout')
            return
        if self.control_special():
            return
        c = self.config
        if self.state in self.navigation_states:
            self.navigate()
            return
        if self.state == 'FACE_SHELF':
            error = wrap(c['shelf_heading']-self.heading)
            if abs(error) < .025:
                self.safe_command()
                self.transition('ACQUIRE_VIEW')
            else:
                self.safe_command(0., clamp(1.5*error, c['angular_speed']))
            return
        if self.state == 'ACQUIRE_VIEW':
            self.safe_command()
            if self.errors() is not None and wall-self.view_received < .8:
                self.transition('APPROACH')
            elif wall-self.since > 8:
                reason = self.observation.reason if self.observation else 'No image observation'
                self.transition('FAILED', 'Shelf acquisition: '+reason)
            return
        errors = self.errors()
        if errors is None or wall-self.view_received > .8:
            self.safe_command()
            if self.state != 'INSPECT':
                self.transition('ACQUIRE_VIEW')
            elif wall-self.since > 12:
                self.transition('FAILED', 'Inspection view remained unreliable')
            return
        self.final_errors = heading, distance, lateral = errors
        if self.state == 'BACK_OFF':
            if self.line.distance >= c['inspection_distance']+.20:
                self.safe_command()
                self.transition('APPROACH')
            elif abs(heading) > .04:
                self.safe_command(0., clamp(1.5*heading, .18))
            else:
                self.safe_command(-.055, clamp(heading, .10))
        elif self.state == 'APPROACH':
            if distance < -.05 or (distance <= .05 and abs(lateral) > .075):
                self.backoffs += 1
                if self.backoffs > 2:
                    self.transition('FAILED', 'Lateral alignment did not converge')
                else:
                    self.transition('BACK_OFF')
                self.safe_command()
            elif distance <= .05:
                self.safe_command()
                self.transition('ALIGN')
            else:
                offset = clamp(atan2(lateral, max(distance, .20)), .22)
                error = heading+offset
                speed = min(c['approach_speed'], max(.025, distance*.45)) if abs(error) < .18 else 0.
                self.safe_command(speed, clamp(1.5*error, .25))
        elif self.state == 'ALIGN':
            if abs(distance) > .07 or abs(lateral) > .08:
                self.transition('APPROACH')
                self.safe_command()
            elif abs(heading) < .025:
                self.safe_command()
                self.transition('SETTLE')
            else:
                self.safe_command(0., clamp(1.5*heading, .16))
        elif self.state == 'SETTLE':
            self.safe_command()
            if abs(heading) > .05 or abs(distance) > .08 or abs(lateral) > .09:
                self.transition('APPROACH')
            elif self.stationary():
                if self.stable_since is None:
                    self.stable_since = wall
                if wall-self.stable_since >= c['stable_seconds']:
                    self.last_counts = self.last_center = None
                    self.transition('INSPECT')
            else:
                self.stable_since = None
            if wall-self.since > 10:
                self.transition('FAILED', 'Robot did not settle')
        elif self.state == 'INSPECT':
            self.safe_command()
            if not self.stationary():
                self.transition('SETTLE')
            elif wall-self.since > 12:
                self.transition('FAILED', 'Image counts did not stabilize')

    def process_image(self):
        if self.state not in self.viewing or 'image' not in self.latest:
            return
        match = synchronized_image(self.image_history, self.lines,
                                   self.get_clock().now().nanoseconds*1e-9)
        if match is None:
            return
        received, msg, line = match
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.processed_stamp:
            return
        self.processed_stamp = stamp
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
        self.observation = observation = detect_boxes(rgb)
        self.view_received = received
        self.view_lateral = None
        if observation.valid:
            info = self.latest['info'][1]
            self.view_lateral = camera_lateral(observation.center_x, info.k[0], info.k[2],
                                               line, *self.mounts)
            self.view_position = tuple(self.position)
        if self.state == 'INSPECT':
            if observation.valid and self.stationary() and self.errors() is not None:
                center = observation.center_x
                consistent = (observation.counts == self.last_counts and self.last_center is not None
                              and abs(center-self.last_center) < 2.5)
                self.stable_count = self.stable_count+1 if consistent else 1
                self.last_counts, self.last_center = observation.counts, center
            else:
                self.stable_count = 0
            if self.stable_count >= self.config['stable_frames']:
                self.inspection_complete()
        annotated = annotate(rgb, observation, self.stable_count)
        image = Image(header=msg.header, height=msg.height, width=msg.width,
                      encoding='bgr8', step=msg.width*3, data=array('B', annotated.tobytes()))
        self.images.publish(image)

    def control_special(self):
        return False

    def navigate(self):
        c = self.config
        staging_y = c['station_front_y']-c['inspection_distance']-.15-self.mounts[1]
        targets = ((c['station_x']-.65, staging_y), (c['station_x'], staging_y))
        target = targets[self.waypoint]
        dx, dy = target[0]-self.position[0], target[1]-self.position[1]
        distance = hypot(dx, dy)
        if distance < .05:
            self.safe_command()
            self.waypoint += 1
            if self.waypoint == len(targets):
                self.transition('FACE_SHELF')
            return
        error = wrap(atan2(dy, dx)-self.heading)
        speed = min(c['linear_speed'], max(.035, distance*.7)) if abs(error) < .18 else 0.
        self.safe_command(speed, clamp(1.6*error, c['angular_speed']))

    def inspection_complete(self):
        self.transition('COMPLETE', 'Stable two-level camera inspection')

    def publish_result(self, success):
        if self.result_sent:
            return
        result = InspectionResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = 'camera_optical_frame'
        result.success, result.status, result.detail = success, self.state, self.detail
        if success:
            result.upper_red, result.upper_blue, result.lower_red, result.lower_blue = self.observation.counts
            result.total_visible = sum(self.observation.counts)
        result.heading_error, result.distance_error, result.lateral_error = map(float, self.final_errors)
        result.stable_frames = self.stable_count
        self.results.publish(result)
        self.result_sent = True
        self.get_logger().info(f'Inspection {result.status}: upper red={result.upper_red} blue={result.upper_blue}; lower red={result.lower_red} blue={result.lower_blue}; total={result.total_visible}')

    def publish_status(self):
        self.states.publish(String(data=self.state))
        status = DiagnosticStatus(name='inventory_mission', hardware_id='warehouse_robot',
                                  level=DiagnosticStatus.ERROR if self.state in ('FAILED', 'BLOCKED', 'SENSOR_TIMEOUT') else DiagnosticStatus.OK,
                                  message=self.state+(': '+self.detail if self.detail else ''))
        values = dict(heading_error_rad=self.final_errors[0], distance_error_m=self.final_errors[1],
                      lateral_error_m=self.final_errors[2], estimated_x=self.position[0],
                      estimated_y=self.position[1], stable_frames=self.stable_count)
        if self.line:
            values.update(scan_inliers=self.line.inliers, scan_span_m=self.line.span,
                          scan_fit_residual_m=self.line.residual, shelf_distance_m=self.line.distance)
        if self.observation:
            values.update(view_valid=self.observation.valid, view_reason=self.observation.reason)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = InventoryMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command.publish(Twist())
        node.listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
