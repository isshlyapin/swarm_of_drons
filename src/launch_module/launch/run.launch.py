import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('launch_module'), 'launch'
    )

    launch_files = [
        'simulation_clock.launch.py',
        'drone_composition.launch.py',
        'drone_visual_controller.launch.py',
        'map_visual.launch.py',
        'rviz.launch.py',
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, file))
        )
        for file in launch_files
    ]

    return LaunchDescription(includes)
