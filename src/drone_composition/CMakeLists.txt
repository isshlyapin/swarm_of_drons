cmake_minimum_required(VERSION 3.8)
project(drone_composition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === find dependencies ===
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(data_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(navigator_interfaces REQUIRED)

find_package(magic_enum CONFIG REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(node_plugins "")

# === Drone component ===
add_library(drone SHARED
    src/drone.cpp
)
target_include_directories(drone 
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(drone
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(drone
    rclcpp
    Eigen3
    nav_msgs
    drone_interfaces
    rclcpp_components
)
target_link_libraries(drone
    magic_enum::magic_enum
)
rclcpp_components_register_nodes(drone "DroneComposition::Drone")
set(node_plugins "${node_plugins}DroneComposition::Drone;$<TARGET_FILE:drone>\n")


# === Drone controller component ===
add_library(drone_controller SHARED
    src/drone_controller.cpp
)
target_include_directories(drone_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(drone_controller
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(drone_controller
    rclcpp
    Eigen3
    std_msgs
    std_srvs
    nav_msgs
    data_utils
    geometry_msgs
    drone_interfaces
    rclcpp_components
    navigator_interfaces
)
target_link_libraries(drone_controller
    magic_enum::magic_enum
)
rclcpp_components_register_nodes(drone_controller "DroneComposition::DroneController")
set(node_plugins "${node_plugins}DroneComposition::DroneController;$<TARGET_FILE:drone_controller>\n")


# === Install and Export ===
ament_export_targets(export_drone HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    Eigen3
    nav_msgs
    drone_interfaces
    rclcpp_components
)

install(
    TARGETS drone
    EXPORT export_drone
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS drone_controller
    EXPORT export_drone_controller
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()