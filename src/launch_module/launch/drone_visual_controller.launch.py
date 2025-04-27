from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    drones_file_arg = DeclareLaunchArgument(
        'drones_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test2/drones.csv'
        )
    )

    return LaunchDescription([
        drones_file_arg,
        Node(
            package='drones_visualization',
            name='visualization1',
            executable='visualization',
            parameters = [{
                'use_sim_time': True,
                'drones_file': LaunchConfiguration('drones_file')
            }],
        )
    ])