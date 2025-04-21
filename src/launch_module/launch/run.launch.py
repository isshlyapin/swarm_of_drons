import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    simulation_clock_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_module'), 'launch'),
            '/simulation_clock.launch.py'])
    )
    drone_controller1_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_module'), 'launch'),
            '/drone_controller.launch.py'])
    )
    drone_visual_controller1_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_module'), 'launch'),
            '/drone_visual_controller.launch.py'])
    )
    mission_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_module'), 'launch'),
            '/mission_publisher.launch.py'])
    )
    map_visual_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_module'), 'launch'),
            '/map_visual.launch.py'])
    )
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_module'), 'launch'),
            '/rviz.launch.py'])
    )

    return LaunchDescription([
        simulation_clock_node,
        drone_controller1_node,
        drone_visual_controller1_node,
        # mission_publisher_node,
        map_visual_node,
        rviz_node
    ])