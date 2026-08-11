cmake_minimum_required(VERSION 3.8)
project(rover_arm_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(sim_control_node src/sim_control_node.cpp)
ament_target_dependencies(sim_control_node rclcpp sensor_msgs trajectory_msgs)

add_executable(real_arm_teleop_node src/real_arm_teleop_node.cpp)
ament_target_dependencies(real_arm_teleop_node rclcpp sensor_msgs trajectory_msgs std_msgs)

add_executable(moveit_goal_node src/moveit_goal_node.cpp)

ament_target_dependencies(
  moveit_goal_node
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
install(TARGETS
  sim_control_node
  real_arm_teleop_node  
  moveit_goal_node
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY urdf stl launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()