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
find_package(geometry_msgs REQUIRED)

add_executable(spawner src/turtle_spawner.cpp )
ament_target_dependencies(spawner rclcpp turtlesim my_robot_interfaces)

add_executable(controller src/turtle_controller.cpp )
ament_target_dependencies(controller rclcpp turtlesim my_robot_interfaces geometry_msgs)

install(TARGETS 
spawner 
controller
DESTINATION lib/${PROJECT_NAME}
)
ament_package()
