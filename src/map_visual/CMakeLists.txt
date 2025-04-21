cmake_minimum_required(VERSION 3.8)
project(map_visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(map_publisher_node src/main.cpp)
ament_target_dependencies(map_publisher_node 
    rclcpp 
    visualization_msgs
)

install(TARGETS 
    map_publisher_node 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
