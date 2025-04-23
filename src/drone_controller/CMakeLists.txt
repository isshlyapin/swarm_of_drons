cmake_minimum_required(VERSION 3.8)
project(drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(data_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(navigator_interfaces REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(magic_enum CONFIG REQUIRED)

add_executable(drone_controller 
    src/main.cpp
    src/drone.cpp
    src/drone_controller.cpp
)

target_include_directories(drone_controller PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(drone_controller 
  magic_enum::magic_enum
)

ament_target_dependencies(drone_controller
    rclcpp
    Eigen3
    std_msgs
    nav_msgs
    std_srvs
    data_utils
    geometry_msgs
    drone_interfaces
    navigator_interfaces
)

install(TARGETS
    drone_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
