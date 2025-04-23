cmake_minimum_required(VERSION 3.8)
project(map_visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(data_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(map_publisher_node 
    src/main.cpp
    src/marker_publisher.cpp
)
target_include_directories(map_publisher_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_publisher_node 
    rclcpp 
    Eigen3
    data_utils
    visualization_msgs
)

install(TARGETS 
    map_publisher_node 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
