cmake_minimum_required(VERSION 3.8)
project(drones_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(data_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(drone_composition REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(magic_enum CONFIG REQUIRED)

add_executable(visualization 
    src/main.cpp
    src/drone_visual.cpp
    src/drone_visual_controller.cpp
)
target_link_libraries(visualization 
    magic_enum::magic_enum
)
target_include_directories(visualization PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)
ament_target_dependencies(visualization
    rclcpp 
    nav_msgs
    data_utils
    geometry_msgs
    drone_interfaces
    drone_composition
    visualization_msgs
)

install(TARGETS 
    visualization
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
