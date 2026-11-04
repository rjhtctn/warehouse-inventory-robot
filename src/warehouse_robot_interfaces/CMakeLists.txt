cmake_minimum_required(VERSION 3.20)
project(warehouse_robot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InspectionResult.msg" "msg/ShelfInventory.msg" "msg/PatrolResult.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
