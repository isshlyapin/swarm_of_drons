cmake_minimum_required(VERSION 3.8)
project(navigator2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(data_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(drone_composition REQUIRED)
find_package(navigator_interfaces REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(navigator2_lib SHARED
    src/map.cpp
    src/mission_manager.cpp
    src/navigator.cpp
)
target_include_directories(navigator2_lib
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(navigator2_lib
    Eigen3
    rclcpp
    data_utils
    geometry_msgs
    drone_interfaces
    navigator_interfaces
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS navigator2_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
    eigen3_cmake_module
    Eigen3
    rclcpp
    data_utils
    geometry_msgs
    drone_interfaces
    navigator_interfaces
)

add_executable(navigator2
    src/main.cpp
)
ament_target_dependencies(navigator2
    rclcpp
)
target_link_libraries(navigator2
    navigator2_lib
)
install(TARGETS
    navigator2
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    ament_add_gtest(test_map
        test/test_map.cpp
    )
    target_link_libraries(test_map
        navigator2_lib
    )

    ament_add_gtest(test_interface
        test/test_config_load.cpp    
    )
    target_link_libraries(test_interface
        navigator2_lib
    )

    # Integration tests
    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    function(add_ros_isolated_launch_test path)
        set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
        add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
    endfunction()
    add_ros_isolated_launch_test(test/test_execute_mission.py)
    add_ros_isolated_launch_test(test/test_relocate_execute_mission.py)
endif()

ament_package()
