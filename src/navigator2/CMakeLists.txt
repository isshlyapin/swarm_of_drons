cmake_minimum_required(VERSION 3.8)
project(navigator2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(data_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(navigator_interfaces REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(navigator2
    src/map.cpp
    src/mission_manager.cpp
    src/navigator.cpp
    src/main.cpp
)

target_include_directories(navigator2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(navigator2
    Eigen3
    rclcpp
    geometry_msgs
    data_utils
    drone_interfaces
    navigator_interfaces
)

install(TARGETS
    navigator2
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    ament_add_gtest(test_map
        src/map.cpp
        test/test1.cpp
    )
    ament_target_dependencies(test_map
        rclcpp
        Eigen3
        data_utils
        geometry_msgs
        drone_interfaces
        navigator_interfaces
    )
    target_include_directories(test_map PRIVATE include)
endif()

ament_package()
