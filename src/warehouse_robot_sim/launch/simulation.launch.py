import os
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess
from launch.actions import LogError, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def stop_world(context):
    try:
        subprocess.run(
            ['gz', 'service', '-s', '/world/warehouse/control',
             '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000', '--req', 'pause: true'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2, check=False)
        subprocess.run(
            ['gz', 'service', '-s', '/server_control',
             '--reqtype', 'gz.msgs.ServerControl', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000', '--req', 'stop: true'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2, check=False)
    except (OSError, subprocess.TimeoutExpired):
        pass


def launch_system(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    enabled = lambda name: value(name).lower() == 'true'
    sim = get_package_share_directory('warehouse_robot_sim')
    description = get_package_share_directory('warehouse_robot_description')
    control = get_package_share_directory('warehouse_robot_control')
    robot = xacro.process_file(
        os.path.join(description, 'urdf', 'warehouse_robot.urdf.xacro'),
        mappings={name: value(name) for name in ('lidar', 'camera', 'imu')}).toxml()
    use_sim_time = {'use_sim_time': True}
    autonomous = enabled('autonomous')
    inspection = enabled('inspection')
    patrol = enabled('patrol')
    if sum((autonomous, inspection, patrol)) > 1:
        raise RuntimeError('Select one motion mode: autonomous, inspection, or patrol')
    if (inspection or patrol) and (value('world') != 'warehouse' or not all(enabled(n) for n in ('lidar', 'camera', 'imu'))):
        raise RuntimeError('Inspection requires the warehouse and all three sensors')
    if autonomous and not enabled('lidar'):
        raise RuntimeError('Autonomous operation requires the LiDAR')
    world = os.path.join(sim, 'worlds', value('world') + '.sdf')
    render_environment = {'LP_NUM_THREADS': os.environ.get('LP_NUM_THREADS', '1')}
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '2', world],
        name='gazebo', output='screen', sigterm_timeout='15',
        additional_env=render_environment,
        on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])
    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                           parameters=[use_sim_time, {'robot_description': robot}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 parameters=[{'world': 'warehouse', 'name': 'warehouse_robot',
                              'topic': '/robot_description', 'allow_renaming': False,
                              'x': float(value('x')), 'y': float(value('y')), 'z': 0.005,
                              'Y': float(value('yaw'))}])
    obstacles = Node(package='ros_gz_sim', executable='create', output='screen',
                     arguments=['-world', 'warehouse', '-file',
                                os.path.join(sim, 'worlds', 'route_obstacles.sdf')])
    bridge = Node(package='warehouse_robot_sim', executable='simulation_bridge', name='ros_gz_bridge',
                  parameters=[use_sim_time, {'config_file': os.path.join(sim, 'config', 'bridge.yaml')}],
                  output='screen')
    params = os.path.join(control, 'config', 'control.yaml')
    guard = Node(package='warehouse_robot_control', executable='motion_guard', name='motion_guard',
                 parameters=[params, use_sim_time, {'autonomous': autonomous,
                             'controller': 'patrol' if patrol else 'inspection' if inspection else 'avoidance' if autonomous else 'manual'}], output='screen')
    components = [guard]
    if enabled('gui'):
        gui = os.path.join(get_package_prefix('gz_sim_vendor'), 'opt', 'gz_sim_vendor',
                           'libexec', 'gz', 'sim10', 'gz-sim-gui-client')
        if not os.path.isfile(gui):
            raise RuntimeError(f'Gazebo Jetty GUI executable not found: {gui}')
        components.append(ExecuteProcess(
            cmd=[gui, '--verbose', '2', '--gui-config', os.path.join(sim, 'config', 'warehouse.gui')],
            name='gazebo_gui', output='screen',
            additional_env=render_environment,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo GUI exited'))]))
    if enabled('camera'):
        components.append(Node(package='ros_gz_image', executable='image_bridge',
                               name='camera_bridge', arguments=['/camera/image_raw'],
                               parameters=[use_sim_time, {'qos': 'sensor_data'}], output='screen'))
    if enabled('monitors'):
        for sensor in ('lidar', 'camera', 'imu'):
            if enabled(sensor):
                components.append(Node(package='warehouse_robot_control', executable='sensor_monitor',
                                       name=f'{sensor}_monitor', output='screen',
                                       parameters=[use_sim_time, {'sensor': sensor}]))
    if autonomous:
        components.append(Node(package='warehouse_robot_control', executable='obstacle_avoidance',
                               name='obstacle_avoidance', parameters=[params, use_sim_time], output='screen'))
    if inspection:
        components.append(Node(package='warehouse_robot_control', executable='inventory_mission',
                               name='inventory_mission', parameters=[
                                   os.path.join(control, 'config', 'inspection.yaml'), use_sim_time,
                                   {'initial_x': float(value('x')), 'initial_y': float(value('y'))}],
                               output='screen',
                               on_exit=[EmitEvent(event=Shutdown(reason='Inspection controller exited'))]))
    if patrol:
        components.append(Node(package='warehouse_robot_control', executable='inventory_patrol',
                               name='inventory_patrol', parameters=[
                                   os.path.join(control, 'config', 'patrol.yaml'), use_sim_time,
                                   {'initial_x': float(value('x')), 'initial_y': float(value('y')),
                                    'reactive_avoidance': enabled('reactive')}],
                               output='screen',
                               on_exit=[EmitEvent(event=Shutdown(reason='Patrol controller exited'))]))
    if enabled('rviz'):
        components.append(Node(package='warehouse_robot_sim', executable='robot_view', name='rviz2',
                               parameters=[use_sim_time], output='screen',
                               arguments=['-d', os.path.join(sim, 'config', 'warehouse.rviz')],
                               additional_env={**render_environment, 'QT_QPA_PLATFORM': 'xcb'}))

    def after_spawn(event, context):
        if event.returncode != 0:
            return [LogError(msg='Robot spawning failed'), EmitEvent(event=Shutdown(reason='Spawn failed'))]
        return [obstacles] if patrol and enabled('obstacles') else components

    def after_obstacles(event, context):
        if event.returncode != 0:
            return [LogError(msg='Obstacle spawning failed'), EmitEvent(event=Shutdown(reason='Obstacle spawning failed'))]
        return components

    def guard_exit(event, context):
        return [EmitEvent(event=Shutdown(reason='Motion guard exited'))]

    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_world)])),
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
            RegisterEventHandler(OnProcessExit(target_action=obstacles, on_exit=after_obstacles)),
            RegisterEventHandler(OnProcessExit(target_action=guard, on_exit=guard_exit)),
            gazebo, state_publisher, bridge, spawn]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('autonomous', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('inspection', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('patrol', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('reactive', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('obstacles', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='warehouse', choices=['warehouse', 'test']),
        DeclareLaunchArgument('x', default_value='-2.8'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    for name in ('gui', 'rviz', 'monitors', 'lidar', 'camera', 'imu'):
        arguments.append(DeclareLaunchArgument(name, default_value='true', choices=['true', 'false']))
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_system)])
