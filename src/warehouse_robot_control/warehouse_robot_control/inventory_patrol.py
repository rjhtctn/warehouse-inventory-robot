from math import atan2, cos, hypot, isfinite, pi, sin
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from warehouse_robot_interfaces.msg import InspectionResult, PatrolResult, ShelfInventory

from warehouse_robot_control.inspection_geometry import wrap
from warehouse_robot_control.inventory_mission import InventoryMission, clamp
from warehouse_robot_control.patrol_geometry import home_wall_position, waypoints
from warehouse_robot_control.local_avoidance import ScanPlanner
from warehouse_robot_control.inspection_geometry import sector_clearance


class InventoryPatrol(InventoryMission):
    terminal = ('COMPLETE', 'BLOCKED', 'FAILED', 'INSPECTION_FAILED')
    navigation_states = ('NAVIGATE', 'DETOUR', 'RETURN_HOME')

    def __init__(self):
        super().__init__('inventory_patrol', '/patrol/inspection')
        self.station_ids = list(self.declare_parameter('station_ids', ['north_west', 'south_west']).value)
        xs = self.declare_parameter('station_xs', [-1.45, -1.45]).value
        ys = self.declare_parameter('station_front_ys', [1.64, -1.64]).value
        headings = self.declare_parameter('station_headings', [pi/2, -pi/2]).value
        if (len(set(self.station_ids)) != len(self.station_ids) or not self.station_ids
                or not all(len(v) == len(self.station_ids) for v in (xs, ys, headings))
                or not all(isfinite(v) for values in (xs, ys, headings) for v in values)):
            raise ValueError('Station identifiers and poses must be unique and complete')
        self.stations = list(zip(xs, ys, headings))
        self.routes = [waypoints(self.declare_parameter('route_'+name, [x-.65, y-sin(h)*2.59,
                                                                                 x, y-sin(h)*2.59]).value)
                       for name, (x, y, h) in zip(self.station_ids, self.stations)]
        self.home = (self.config['initial_x'], self.config['initial_y'])
        self.home_heading = self.declare_parameter('home_heading', 0.).value
        self.home_route = waypoints(self.declare_parameter('route_home', [-2.8, .9, *self.home]).value)
        self.walls = tuple(self.declare_parameter(k, v).value for k, v in
                           (('west_wall', -3.94), ('north_wall', 2.94), ('south_wall', -2.94)))
        self.waypoint_timeout = self.declare_parameter('waypoint_timeout', 30.).value
        self.progress_timeout = self.declare_parameter('progress_timeout', 15.).value
        if not 0 < self.progress_timeout < self.waypoint_timeout:
            raise ValueError('Invalid navigation timeouts')
        self.station_index = 0
        self.completed = {}
        self.active_route = self.routes[0]
        self.nav_since = self.progress_since = time.monotonic()
        self.best_distance = float('inf')
        self.home_position_error = self.home_heading_error = float('nan')
        self.home_fix_received = 0.
        self.last_home_stamp = None
        self.home_fix = None
        self.recorded_at = 0.
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.patrol_results = self.create_publisher(PatrolResult, '/patrol/result', retained)
        self.shelf_results = self.create_publisher(ShelfInventory, '/patrol/shelf',
                         QoSProfile(depth=max(1, len(self.station_ids)), durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.patrol_states = self.create_publisher(String, '/patrol/state', retained)
        self.patrol_diagnostics = self.create_publisher(DiagnosticArray, '/patrol/diagnostics', 5)
        self.route_publisher = self.create_publisher(Path, '/patrol/route', retained)
        self.path_publisher = self.create_publisher(Path, '/patrol/path', retained)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.select_station(0)
        self.reactive = self.declare_parameter('reactive_avoidance', True).value
        self.avoid_timeout = self.declare_parameter('avoid_timeout', 65.).value
        self.planner = ScanPlanner()
        self.plan = None
        self.plan_time = 0.
        self.resume_state = None
        self.avoid_since = 0.
        self.hit_distance = 0.
        self.avoid_count = 0
        self.avoid_attempts = {}
        self.plan_failures = 0
        self.clear_scans = 0
        self.last_clear_stamp = None
        self.planner_received = 0.

    def receive(self, name, msg):
        super().receive(name, msg)
        if (name == 'scan' and hasattr(self, 'planner') and self.reactive and self.mounts is not None
                and self.heading is not None and self.scan_summary is not None
                and self.state in self.navigation_states+('BRAKE', 'AVOID_OBSTACLE', 'RECOVER_ROUTE')):
            self.planner.update(msg, self.position, self.heading, self.mounts[1])
            self.planner_received = time.monotonic()

    def path_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x-self.home[0], y-self.home[1]
        pose.pose.orientation.w = 1.
        return pose

    def select_station(self, index):
        self.station_index = index
        x, y, heading = self.stations[index]
        self.config.update(station_x=x, station_front_y=y, shelf_heading=heading)
        self.active_route = self.routes[index]
        self.waypoint = 0
        self.lines.clear()
        self.image_history.clear()
        self.line = self.observation = self.view_lateral = None
        self.backoffs = self.stable_count = 0
        self.last_counts = self.last_center = self.processed_stamp = None
        self.reset_progress()

    def reset_progress(self):
        self.nav_since = self.progress_since = time.monotonic()
        self.best_distance = float('inf')

    def transition(self, state, detail=''):
        if state == 'FAILED' and self.state in self.viewing:
            state = 'INSPECTION_FAILED'
        super().transition(state, detail)

    def navigate(self):
        if self.state == 'RETURN_HOME':
            self.update_home_fix()
        target = self.active_route[self.waypoint]
        dx, dy = target[0]-self.position[0], target[1]-self.position[1]
        distance = hypot(dx, dy)
        now = time.monotonic()
        if self.reactive and distance > .10:
            look_distance = min(distance, .95)
            end = (self.position[0]+dx/distance*look_distance,
                   self.position[1]+dy/distance*look_distance)
            if self.planner.grid is None or now-self.planner_received > .3:
                self.safe_command()
                return
            if not self.planner.segment_clear(self.position, end, .50):
                self.begin_avoidance(distance)
                return
        if distance < self.best_distance-.025:
            self.best_distance, self.progress_since = distance, now
        if now-self.nav_since > self.waypoint_timeout or now-self.progress_since > self.progress_timeout:
            self.transition('FAILED', 'Waypoint progress timeout')
            return
        if distance < .045:
            self.safe_command()
            self.waypoint += 1
            self.reset_progress()
            if self.waypoint == len(self.active_route):
                self.transition('HOME_ALIGN' if self.state == 'RETURN_HOME' else 'FACE_SHELF')
            elif self.state == 'DETOUR' and self.waypoint == len(self.active_route)-1:
                self.transition('NAVIGATE', 'Detour cleared; resuming station approach')
            return
        error = wrap(atan2(dy, dx)-self.heading)
        speed = min(self.config['linear_speed'], max(.03, distance*.7)) if abs(error) < .16 else 0.
        if self.scan_summary.nearest < .60:
            speed = min(speed, .09)
        self.safe_command(speed, clamp(1.6*error, self.config['angular_speed']))

    def begin_avoidance(self, distance):
        key = self.station_index, self.state, self.waypoint
        attempts = self.avoid_attempts.get(key, 0)+1
        self.avoid_attempts[key] = attempts
        if attempts > 3:
            self.transition('BLOCKED', 'Bounded obstacle recovery attempts exhausted')
            return
        self.resume_state = self.state
        self.hit_distance = distance
        self.hit_position = tuple(self.position)
        self.avoid_since = time.monotonic()
        self.plan = None
        self.plan_failures = self.clear_scans = 0
        self.transition('BRAKE', 'Live scan obstructs active waypoint corridor')
        self.safe_command()

    def avoid_step(self):
        now = time.monotonic()
        goal = self.active_route[self.waypoint]
        if now-self.avoid_since > self.avoid_timeout:
            self.transition('BLOCKED', 'Local avoidance progress timeout')
            return
        if self.scan_summary.nearest < .38:
            self.transition('BLOCKED', 'Insufficient clearance for safe obstacle recovery')
            return
        distance = hypot(goal[0]-self.position[0], goal[1]-self.position[1])
        progress = (distance < self.hit_distance-.12
                    and hypot(self.position[0]-self.hit_position[0], self.position[1]-self.hit_position[1]) > .35)
        clear = self.planner.segment_clear(self.position, goal, .50)
        stamp = self.latest['scan'][1].header.stamp
        stamp = stamp.sec+stamp.nanosec*1e-9
        if stamp != self.last_clear_stamp:
            self.clear_scans = self.clear_scans+1 if clear and progress else 0
            self.last_clear_stamp = stamp
        if self.clear_scans >= 3:
            if self.state != 'RECOVER_ROUTE':
                self.transition('RECOVER_ROUTE', 'Fresh scans confirm clear goal corridor and progress')
            error = wrap(atan2(goal[1]-self.position[1], goal[0]-self.position[0])-self.heading)
            if abs(error) < .10:
                self.avoid_count += 1
                self.reset_progress()
                self.transition(self.resume_state, 'Reactive obstacle recovery completed; original waypoint retained')
                self.safe_command()
            else:
                self.safe_command(0., clamp(1.5*error, .35))
            return
        if self.state == 'RECOVER_ROUTE':
            self.transition('AVOID_OBSTACLE', 'Goal corridor became obstructed again')
        if self.state == 'BRAKE':
            self.safe_command()
            if not self.stationary() or now-self.since < .35:
                return
        target = self.planner.lookahead(self.position, self.plan.points) if self.plan else None
        if target is None and now-self.plan_time > .5:
            self.plan = self.planner.plan(self.position, goal)
            self.plan_time = now
            if self.plan is None:
                self.plan_failures += 1
                self.safe_command()
                if self.plan_failures >= 4:
                    self.transition('BLOCKED', 'No footprint-clear path in live local scan')
                return
            self.plan_failures = 0
            target = self.planner.lookahead(self.position, self.plan.points)
        if target is None:
            self.safe_command()
            return
        if self.state == 'BRAKE':
            self.transition('AVOID_OBSTACLE', 'Following a temporary path computed from live LiDAR')
        error = wrap(atan2(target[1]-self.position[1], target[0]-self.position[0])-self.heading)
        front = sector_clearance(self.latest['scan'][1], 0.)
        speed = .09 if abs(error) < .18 and front > .56 else 0.
        self.safe_command(speed, clamp(1.6*error, .35))

    def inspection_complete(self):
        if (not self.stationary() or self.stale_sensors() or abs(self.final_errors[0]) > pi/36
                or any(abs(v) > .1 for v in self.final_errors[1:])):
            self.transition('INSPECTION_FAILED', 'Final inspection accuracy was not verified')
            return
        identifier = self.station_ids[self.station_index]
        if identifier in self.completed:
            self.transition('FAILED', 'Duplicate station completion')
            return
        inspection = InspectionResult(success=True, status='COMPLETE',
                                      detail='Stable two-level camera inspection')
        inspection.header.stamp = self.get_clock().now().to_msg()
        inspection.header.frame_id = 'camera_optical_frame'
        inspection.upper_red, inspection.upper_blue, inspection.lower_red, inspection.lower_blue = self.observation.counts
        inspection.total_visible = sum(self.observation.counts)
        inspection.heading_error, inspection.distance_error, inspection.lateral_error = map(float, self.final_errors)
        inspection.stable_frames = self.stable_count
        shelf = ShelfInventory(shelf_id=identifier, inspection=inspection)
        self.completed[identifier] = shelf
        self.shelf_results.publish(shelf)
        self.recorded_at = time.monotonic()
        self.get_logger().info(f'Recorded {identifier}: upper={self.observation.counts[:2]} lower={self.observation.counts[2:]} total={inspection.total_visible}')
        self.transition('RECORD', identifier)
        self.safe_command()

    def control_special(self):
        if self.state in ('BRAKE', 'AVOID_OBSTACLE', 'RECOVER_ROUTE'):
            self.avoid_step()
            return True
        if self.state == 'RECORD':
            self.safe_command()
            if time.monotonic()-self.recorded_at >= .4:
                self.transition('NEXT_STATION')
            return True
        if self.state == 'NEXT_STATION':
            self.safe_command()
            if self.station_index+1 < len(self.station_ids):
                self.select_station(self.station_index+1)
                self.transition('DETOUR', 'Following configured central-obstacle detour')
            else:
                self.active_route = self.home_route
                self.waypoint = 0
                self.reset_progress()
                self.transition('RETURN_HOME')
            return True
        if self.state in ('HOME_ALIGN', 'HOME_VERIFY'):
            self.update_home_fix()
            self.home_position_error = hypot(self.position[0]-self.home[0], self.position[1]-self.home[1])
            self.home_heading_error = wrap(self.home_heading-self.heading)
            if time.monotonic()-self.since > 15:
                self.transition('FAILED', 'Return-home verification timeout')
            elif abs(self.home_heading_error) > .025:
                self.safe_command(0., clamp(1.5*self.home_heading_error, .25))
                if self.state == 'HOME_VERIFY':
                    self.transition('HOME_ALIGN')
            elif time.monotonic()-self.home_fix_received > .5:
                self.safe_command()
            elif self.home_position_error > .055:
                self.active_route = (self.home,)
                self.waypoint = 0
                self.reset_progress()
                self.transition('RETURN_HOME', 'Correcting home position from live wall ranges')
            else:
                self.safe_command()
                if self.state == 'HOME_ALIGN':
                    self.transition('HOME_VERIFY')
                elif self.stationary():
                    if self.stable_since is None:
                        self.stable_since = time.monotonic()
                    if time.monotonic()-self.stable_since > self.config['stable_seconds']:
                        if set(self.completed) == set(self.station_ids):
                            self.transition('COMPLETE', 'All stations inspected and home pose verified')
                        else:
                            self.transition('FAILED', 'Required stations were not completed')
                else:
                    self.stable_since = None
            return True
        return False

    def update_home_fix(self):
        if hypot(self.position[0]-self.home[0], self.position[1]-self.home[1]) > .35:
            return
        scan = self.latest['scan'][1]
        stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if stamp == self.last_home_stamp:
            return
        self.last_home_stamp = stamp
        position = home_wall_position(scan, self.heading, self.mounts[1], *self.walls)
        if position is not None and hypot(position[0]-self.position[0], position[1]-self.position[1]) < .25:
            self.position[:] = position
            self.home_fix = position
            self.home_fix_received = time.monotonic()

    def publish_result(self, success):
        if self.result_sent:
            return
        result = PatrolResult(success=success, status=self.state, detail=self.detail,
                              shelves=list(self.completed.values()), required_stations=len(self.station_ids),
                              completed_stations=len(self.completed), home_position_error=float(self.home_position_error),
                              home_heading_error=float(self.home_heading_error), duration=time.monotonic()-self.started)
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = 'odom'
        result.total_red = sum(s.inspection.upper_red+s.inspection.lower_red for s in result.shelves)
        result.total_blue = sum(s.inspection.upper_blue+s.inspection.lower_blue for s in result.shelves)
        result.total_visible = result.total_red+result.total_blue
        self.patrol_results.publish(result)
        self.result_sent = True
        self.get_logger().info(f'Patrol {result.status}: stations={result.completed_stations}/{result.required_stations} total={result.total_visible} home_error={result.home_position_error:.3f}m heading={result.home_heading_error:.3f}rad')

    def publish_status(self):
        super().publish_status()
        self.patrol_states.publish(String(data=self.state))
        identifier = self.station_ids[self.station_index] if self.state != 'RETURN_HOME' else 'home'
        values = dict(current_station=identifier, completed=','.join(self.completed),
                      remaining=','.join(s for s in self.station_ids if s not in self.completed),
                      active_waypoint=self.waypoint, waypoint_count=len(self.active_route),
                      estimated_x=self.position[0], estimated_y=self.position[1],
                      home_position_error=self.home_position_error, home_heading_error=self.home_heading_error,
                      total_visible=sum(s.inspection.total_visible for s in self.completed.values()),
                      avoided_obstacles=self.avoid_count, recovery_attempts=sum(self.avoid_attempts.values()),
                      local_plan_points=len(self.plan.points) if self.plan else 0,
                      position_source='wall_ranges' if time.monotonic()-self.home_fix_received < .5 else 'wheel_distance_imu')
        status = DiagnosticStatus(name='inventory_patrol', hardware_id='warehouse_robot',
                                  level=DiagnosticStatus.ERROR if self.state in ('FAILED', 'BLOCKED', 'INSPECTION_FAILED', 'SENSOR_TIMEOUT') else DiagnosticStatus.OK,
                                  message=self.state, values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.patrol_diagnostics.publish(msg)
        route = Path()
        route.header = msg.header
        route.header.frame_id = 'odom'
        route.poses = [self.path_pose(*self.home)]
        for points in self.routes+[self.home_route]:
            route.poses.extend(self.path_pose(*point) for point in points)
        self.route_publisher.publish(route)
        if not self.path.poses or hypot(self.position[0]-self.home[0]-self.path.poses[-1].pose.position.x,
                                      self.position[1]-self.home[1]-self.path.poses[-1].pose.position.y) > .03:
            self.path.poses.append(self.path_pose(*self.position))
        self.path.header.stamp = msg.header.stamp
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = InventoryPatrol()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.safe_command()
        node.listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
