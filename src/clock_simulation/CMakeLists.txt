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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
