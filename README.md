# Autonomous Warehouse Inventory Robot

BIMU4101 - Introduction to Mobile Robotics

A ROS 2 differential-drive robot designed for warehouse inventory inspection.

The robot navigates between shelves, avoids obstacles using LiDAR, detects red and blue boxes through camera-based image processing, and returns to its starting position.

## Environment

- ROS 2 Lyrical
- Gazebo Jetty
- Python and C++
- URDF/Xacro
- OpenCV
- RViz

## Packages

- warehouse_robot_description: Robot model and sensor geometry
- warehouse_robot_sim: Gazebo world, launch and visualization
- warehouse_robot_control: Navigation, obstacle avoidance and inventory detection
- warehouse_robot_interfaces: Custom ROS 2 messages

## Sensors

- LiDAR: /scan
- RGB camera: /camera/image_raw
- IMU: /imu/data
- Odometry: /odom

## Build

From the repository root in a ROS 2 environment:

    source /opt/ros/lyrical/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

## Run

Full warehouse patrol:

    ros2 launch warehouse_robot_sim simulation.launch.py patrol:=true

Manual mode:

    ros2 launch warehouse_robot_sim simulation.launch.py

Obstacle avoidance:

    ros2 launch warehouse_robot_sim simulation.launch.py autonomous:=true

Shelf inspection:

    ros2 launch warehouse_robot_sim simulation.launch.py inspection:=true

## Tests

    colcon test --event-handlers console_direct+
    colcon test-result --verbose
