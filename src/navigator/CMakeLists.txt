cmake_minimum_required(VERSION 3.8)
project(navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(navigator_interfaces REQUIRED)

add_executable(navigator 
  src/main.cpp
  src/graph.cpp 
  src/time_table.cpp 
  src/interface.cpp
)

ament_target_dependencies(navigator
  rclcpp
  drone_interfaces
  navigator_interfaces
)

install(TARGETS 
  navigator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
