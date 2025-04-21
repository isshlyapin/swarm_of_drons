cmake_minimum_required(VERSION 3.8)
project(drones_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)

find_package(magic_enum CONFIG REQUIRED)


add_executable(visualization src/main.cpp)
target_link_libraries(visualization 
  magic_enum::magic_enum
)
target_include_directories(visualization PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)
ament_target_dependencies(visualization
    rclcpp 
    visualization_msgs
    nav_msgs
    geometry_msgs
    drone_interfaces
)

install(TARGETS 
    visualization
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
