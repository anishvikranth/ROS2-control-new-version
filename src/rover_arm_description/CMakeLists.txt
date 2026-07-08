cmake_minimum_required(VERSION 3.8)
project(rover_arm_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(sim_control_node src/sim_control_node.cpp)
ament_target_dependencies(sim_control_node rclcpp sensor_msgs trajectory_msgs)

install(TARGETS
  sim_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf stl launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()