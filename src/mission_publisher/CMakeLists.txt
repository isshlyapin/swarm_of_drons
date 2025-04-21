cmake_minimum_required(VERSION 3.8)
project(mission_publisher)

# Используем C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Включаем предупреждения компилятора
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Находим зависимости
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(data_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)

# Включаем каталоги с заголовочными файлами
include_directories(include)

# Создаем библиотеку
add_library(${PROJECT_NAME}_lib
  src/mission_publisher.cpp
)

# Связываем библиотеку с зависимостями
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  data_utils
  geometry_msgs
  drone_interfaces
)

# Создаем исполняемый файл
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# Устанавливаем исполняемый файл
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Устанавливаем библиотеку
install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Устанавливаем каталог с заголовочными файлами
install(DIRECTORY include/
  DESTINATION include/
)

# Добавляем файлы с миссиями в установку
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../missions/
  DESTINATION share/${PROJECT_NAME}/missions
)

ament_package()