cmake_minimum_required(VERSION 3.8)
project(navigator2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(data_utils REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(navigator_interfaces REQUIRED)
add_executable(navigator2
    src/main.cpp
)

target_include_directories(navigator2 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
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

# if(BUILD_TESTING)
# #   find_package(ament_lint_auto REQUIRED)
# #   ament_lint_auto_find_test_dependencies()

#   # Тесты для Vertex
#   ament_add_gtest(test_vertex test/test1.cpp)
#   ament_target_dependencies(test_vertex
#     Eigen3
# )
# #   target_link_libraries(test_vertex navigator2)
#   target_include_directories(test_vertex PRIVATE include)

# #   # Тесты для Path
# #   ament_add_gtest(test_path test/test_path.cpp)
# #   target_link_libraries(test_path your_package)
  
#   # Аналогично для других тестов...
# endif()

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_test test/test1.cpp)
#   target_include_directories(${PROJECT_NAME}_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
# endif()

ament_package()
