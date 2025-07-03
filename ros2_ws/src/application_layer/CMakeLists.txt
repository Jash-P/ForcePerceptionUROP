cmake_minimum_required(VERSION 3.5)
project(application_layer)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ompl REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Grasp.action"
  "srv/OMPLPlan.srv"
  DEPENDENCIES geometry_msgs trajectory_msgs sensor_msgs
)

# Python executables
install(PROGRAMS
  src/grasp_manager_node.py
  src/motion_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++ OMPL planner
add_executable(ompl_planner_node src/ompl_planner_node.cpp)
ament_target_dependencies(ompl_planner_node
  rclcpp sensor_msgs trajectory_msgs std_msgs ompl
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ompl_planner_node
  ${ompl_LIBRARIES}
)
ament_target_dependencies(ompl_planner_node
  ${cpp_typesupport_target}
)
install(TARGETS ompl_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY action srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()