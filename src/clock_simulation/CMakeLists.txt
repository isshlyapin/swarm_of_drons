cmake_minimum_required(VERSION 3.8)
project(clock_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(clock_simulation_interfaces REQUIRED)

add_executable(sim_clock src/sim_clock.cpp)
target_include_directories(sim_clock
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sim_clock
  rclcpp
  std_srvs
  rosgraph_msgs
  clock_simulation_interfaces
)

install(TARGETS
  sim_clock
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
