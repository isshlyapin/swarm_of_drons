cmake_minimum_required(VERSION 3.8)
project(tmp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(navigator_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(tmp_node src/main.cpp)
ament_target_dependencies(tmp_node
  rclcpp
  navigator_interfaces
)
install(TARGETS
  tmp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
