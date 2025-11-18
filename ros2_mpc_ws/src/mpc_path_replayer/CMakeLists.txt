cmake_minimum_required(VERSION 3.5)
project(mpc_path_replayer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(mpc_path_replayer_node src/mpc_path_replayer_node.cpp)

ament_target_dependencies(mpc_path_replayer_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS
  mpc_path_replayer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
