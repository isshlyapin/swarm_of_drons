# Инструкции для работы с проектом ROS 2 Jazzy (C++)

## Требования
- ROS 2 Jazzy (https://docs.ros.org/en/jazzy/Installation.html)
- CMake >= 3.16
- colcon
- gcc/g++ >= 11
- Python 3.10+ (для сборки и инструментов)

## Сборка проекта

1. Откройте терминал в корне рабочего пространства (где находится папка `src/`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

2. После сборки активируйте окружение:

```bash
source install/setup.bash
```

## Запуск узлов
1. Запуск любого узла:

```bash
ros2 run <package_name> <executable>
```

2. Запуск через launch-файл:

```bash
ros2 launch <package_name> <launch_file.py>
```

## Тестирование

```bash
colcon test --packages-select <package_name>
colcon test  # для всех пакетов
```

## Советы
- Для новых пакетов используйте:
  ```bash
  ros2 pkg create --build-type ament_cmake <package_name>
  ```
- Все зависимости указывайте в `package.xml` и `CMakeLists.txt`.
- Для отладки используйте `RCLCPP_INFO`, `RCLCPP_ERROR` и другие макросы логирования.
- Используйте флаги компилятора: `-Wall -Wextra -Wpedantic`

---

**Контакты:** isshlyapin
