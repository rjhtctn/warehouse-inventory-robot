# Autonomous Warehouse Inventory Robot

BIMU4101 — Introduction to Mobile Robotics

A ROS 2 differential-drive mobile robot developed for warehouse inventory inspection.

The robot follows a predefined route between two shelves, detects nearby obstacles using LiDAR, and calculates local avoidance paths when necessary. At each shelf, it adjusts its position and orientation, identifies red and blue boxes using camera images, and records the inventory count. After inspecting both shelves, the robot returns to its starting position.

The system uses ROS 2 Lyrical and Gazebo Jetty.

## 1. Environment

The project was developed and tested using:

- Ubuntu 26.04 (WSL2)
- ROS 2 Lyrical
- Gazebo Jetty
- Python 3 and C++17
- URDF/Xacro
- OpenCV
- RViz

A graphical desktop environment is required to display Gazebo and RViz. On Windows, the project can run through WSL2 with GUI support.

## 2. Robot Design

The robot uses differential drive with two powered wheels and a rear support caster.

- Chassis dimensions: approximately 46 × 30 cm
- Driven wheel radius: 0.075 m
- Distance between driven wheel centers: 0.36 m
- Drive system: Gazebo DiffDrive
- Robot description: URDF/Xacro

The chassis, wheel arrangement, sensor mounts, and collision geometry are defined in the robot description package.

The simulation contains an 8 × 6 m warehouse with shelves, inventory boxes, and obstacles.

## 3. ROS 2 Packages

| Package | Description |
|---|---|
| `warehouse_robot_description` | URDF/Xacro robot model, geometry, and sensor placement |
| `warehouse_robot_sim` | Gazebo world, simulation launch, sensor bridges, and RViz configuration |
| `warehouse_robot_control` | Navigation, local obstacle avoidance, shelf inspection, sensor monitoring, and motion safety |
| `warehouse_robot_interfaces` | Custom ROS 2 messages for inspection and patrol results |

## 4. Sensors

The robot integrates three sensor types.

| Sensor | ROS 2 topic | Message type | Purpose |
|---|---|---|---|
| LiDAR | `/scan` | `sensor_msgs/msg/LaserScan` | Obstacle detection, distance measurement, and shelf alignment |
| RGB camera | `/camera/image_raw` | `sensor_msgs/msg/Image` | Shelf observation and red/blue box detection |
| IMU | `/imu/data` | `sensor_msgs/msg/Imu` | Orientation and angular velocity measurements |

The robot also uses `/odom` (`nav_msgs/msg/Odometry`) for position and motion estimation.

Sensor data is transferred from Gazebo to ROS 2 through `ros_gz_bridge` and `ros_gz_image`. Measurements can be inspected through ROS 2 topics, sensor monitoring nodes, and RViz.

## 5. Navigation and Inventory Inspection

The warehouse patrol uses predefined waypoints for navigation between the shelves.

LiDAR measurements are used to detect obstacles along the route. When the planned corridor is obstructed, the robot stops and constructs a local occupancy grid from recent scan measurements. A local A* planner searches for a collision-free path around the obstacle.

When the original route becomes accessible again, the robot resumes waypoint navigation. If no safe passage can be found, it stops in the `BLOCKED` state.

At each inspection point, LiDAR, IMU, and camera measurements are used for shelf alignment.

Camera images are processed in HSV color space to detect red and blue boxes. The upper and lower shelf levels are evaluated separately. Inventory results are recorded after eight consistent observations.

In the standard warehouse scene, the robot inspects two shelves containing ten visible boxes: five red and five blue.

This system uses a predefined warehouse route and local obstacle avoidance. It does not perform SLAM or general-purpose global path planning.

## 6. Installation

The following steps assume ROS 2 Lyrical, Gazebo Jetty, `colcon`, and `rosdep` are available on Ubuntu.

Clone the repository:

```bash
git clone https://github.com/rjhtctn/warehouse-inventory-robot.git
cd warehouse-inventory-robot
```

Source the ROS 2 environment:

```bash
source /opt/ros/lyrical/setup.bash
```

Install the dependencies declared by the ROS 2 packages:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

## 7. Running the Simulation

Run the following commands from the repository root after building.

### Full warehouse patrol

```bash
ros2 launch warehouse_robot_sim simulation.launch.py patrol:=true
```

This starts Gazebo, spawns the robot, launches the ROS 2 components, opens RViz, and begins the warehouse patrol.

The robot visits both shelves, counts the boxes, avoids obstacles, and returns to its starting position.

### Manual mode

```bash
ros2 launch warehouse_robot_sim simulation.launch.py
```

The robot starts without an autonomous motion controller. Movement requires external velocity commands published to `/cmd_vel`.

For example, in another terminal with the ROS 2 environment sourced:

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Press `Ctrl+C` to stop publishing. The motion guard stops the robot when valid commands are no longer received.

### Obstacle avoidance mode

```bash
ros2 launch warehouse_robot_sim simulation.launch.py autonomous:=true
```

Starts LiDAR-based reactive obstacle avoidance without the full warehouse inventory patrol.

### Single-shelf inspection mode

```bash
ros2 launch warehouse_robot_sim simulation.launch.py inspection:=true
```

Starts the shelf inspection controller without the full two-shelf patrol.

Only one motion mode should be enabled at a time.

## 8. ROS 2 Topics

| Topic | Description |
|---|---|
| `/cmd_vel` | Velocity commands generated by the controller |
| `/drive/cmd_vel` | Commands forwarded to the Gazebo drive system |
| `/scan` | LiDAR measurements |
| `/camera/image_raw` | RGB camera images |
| `/imu/data` | IMU measurements |
| `/odom` | Robot odometry |
| `/patrol/state` | Current patrol state |
| `/patrol/route` | Planned route |
| `/patrol/path` | Recorded robot path |
| `/patrol/shelf` | Individual shelf inventory results |
| `/patrol/result` | Final patrol result |

To list the active ROS 2 topics:

```bash
ros2 topic list
```

To inspect the robot's patrol state:

```bash
ros2 topic echo /patrol/state --qos-durability transient_local
```

To inspect the final inventory result:

```bash
ros2 topic echo /patrol/result --qos-durability transient_local
```

To check LiDAR measurement frequency:

```bash
ros2 topic hz /scan
```

To inspect IMU measurements:

```bash
ros2 topic echo /imu/data --once
```

## 9. Tests

Run the ROS 2 test suite after building the workspace:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The project includes automated tests for navigation geometry, obstacle avoidance, image processing, and shelf inspection.

## 10. Project Structure

```text
warehouse-inventory-robot/
├── README.md
├── .gitignore
└── src/
    ├── warehouse_robot_description/
    │   ├── urdf/
    │   └── meshes/
    ├── warehouse_robot_sim/
    │   ├── launch/
    │   ├── worlds/
    │   ├── config/
    │   ├── meshes/
    │   └── src/
    ├── warehouse_robot_control/
    │   ├── warehouse_robot_control/
    │   ├── config/
    │   └── test/
    └── warehouse_robot_interfaces/
        └── msg/
```

## 11. Implementation Areas

- Robot and simulation: URDF/Xacro modeling, differential-drive configuration, warehouse environment, camera integration, and RViz visualization.
- Control and algorithms: LiDAR-based navigation, local A* obstacle avoidance, waypoint tracking, shelf alignment, image-based inventory counting, and motion safety.
- Sensor monitoring: LiDAR, RGB camera, and IMU data acquisition and visualization.

The ROS 2 packages and simulation files are maintained in the shared project repository.

## 12. Team Members

| Student | Name | Student ID | GitHub | Main responsibilities |
|---|---|---|---|---|
| Student 1 | Yusuf Emre Gözcü | 1306220090 | [@Parzival51](https://github.com/Parzival51) | LiDAR monitoring, navigation, local A* obstacle avoidance, shelf alignment, and HSV-based box counting |
| Student 2 | Rojhat Çetin | 1306220111 | [@rjhtctn](https://github.com/rjhtctn) | URDF/Xacro robot model, sensor placement, RGB camera integration, Gazebo simulation, and RViz configuration |

Both students contributed to IMU integration, data acquisition, and visualization.
