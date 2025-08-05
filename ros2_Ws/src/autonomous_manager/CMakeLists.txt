cmake_minimum_required(VERSION 3.5)
project(autonomous_manager)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Executable: boustrophedon_node
add_executable(boustrophedon_node src/boustrophedon.cpp)
ament_target_dependencies(boustrophedon_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  mavros_msgs
)

# Install
install(TARGETS
  boustrophedon_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
