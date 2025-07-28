cmake_minimum_required(VERSION 3.5)
project(boundary_mapper)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FenceCorners.msg"
  DEPENDENCIES geometry_msgs
)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
