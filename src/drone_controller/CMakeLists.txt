cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(drone_controller src/main.cpp)
ament_target_dependencies(drone_controller
  rclcpp
  drone_interfaces
  geometry_msgs
  std_msgs
)

install(TARGETS
  drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
