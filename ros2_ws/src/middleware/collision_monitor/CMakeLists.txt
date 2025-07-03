cmake_minimum_required(VERSION 3.5)
project(collision_monitor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CollisionAlert.msg"
  DEPENDENCIES std_msgs
)

# Install message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python node
install(
  PROGRAMS src/collision_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()