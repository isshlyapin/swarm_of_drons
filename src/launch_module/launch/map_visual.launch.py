import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Аргументы запуска
    enable_test_arg = DeclareLaunchArgument(
        'enable_test',
        default_value=TextSubstitution(text='false'),
        description='Enable test mode for map visualization.'
    )

    test_number_arg = DeclareLaunchArgument(
        'test_number',
        default_value=TextSubstitution(text='0'),
        description='Test number (must be > 0 if test mode is enabled).'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test1/graph.csv'
        ),
        description='Path to map CSV file (used only if test mode is disabled).'
    )

    return LaunchDescription([
        enable_test_arg,
        test_number_arg,
        map_file_arg,
        OpaqueFunction(function=launch_map_node)
    ])

def launch_map_node(context, *args, **kwargs):
    enable_test = LaunchConfiguration('enable_test').perform(context).lower() == 'true'
    test_number = LaunchConfiguration('test_number').perform(context)
    default_file = LaunchConfiguration('map_file').perform(context)

    if enable_test:
        try:
            test_num = int(test_number)
            if test_num <= 0:
                raise RuntimeError("test_number must be > 0 when enable_test is true.")
        except ValueError:
            raise RuntimeError(f"Invalid test_number: {test_number}. Must be a positive integer.")

        map_file_path = os.path.join(
            '/workspaces/swarm_of_drons/tests',
            f'test{test_num}',
            'graph.csv'
        )
    else:
        map_file_path = default_file

    return [
        Node(
            package='map_visual',
            executable='map_publisher_node',
            parameters=[{
                'use_sim_time': True,
                'map_file': map_file_path
            }]
        )
    ]
