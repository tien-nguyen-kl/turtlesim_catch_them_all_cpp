cmake_minimum_required(VERSION 3.8)
project(turtlesim_catch_them_all_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(controller src/turtle_controller.cpp )
ament_target_dependencies(controller rclcpp turtlesim geometry_msgs my_robot_interfaces)

ament_package()
