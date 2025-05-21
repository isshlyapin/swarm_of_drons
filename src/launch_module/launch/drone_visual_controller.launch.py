import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Аргументы
    enable_test_arg = DeclareLaunchArgument(
        'enable_test',
        default_value=TextSubstitution(text='false'),
        description='Enable test mode for drone visualization.'
    )

    test_number_arg = DeclareLaunchArgument(
        'test_number',
        default_value=TextSubstitution(text='0'),
        description='Test number (must be > 0).'
    )

    drones_file_arg = DeclareLaunchArgument(
        'drones_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test1/drones.csv'
        ),
        description='Path to drone CSV file (used only if enable_test is false).'
    )

    time_scale_arg = DeclareLaunchArgument(
        'time_scale',
        default_value=TextSubstitution(text='1.0'),
        description='Time scale for simulation.'
    )

    flight_rate_arg = DeclareLaunchArgument(
        'flight_rate',
        default_value=TextSubstitution(text='50.0'),
        description='Flight rate for simulation.'
    )

    return LaunchDescription([
        enable_test_arg,
        test_number_arg,
        drones_file_arg,
        time_scale_arg,
        flight_rate_arg,
        OpaqueFunction(function=launch_visualization)
    ])


def launch_visualization(context, *args, **kwargs):
    enable_test = LaunchConfiguration('enable_test').perform(context).lower() == 'true'
    test_number = LaunchConfiguration('test_number').perform(context)
    default_file = LaunchConfiguration('drones_file').perform(context)
    time_scale = LaunchConfiguration('time_scale').perform(context)
    flight_rate = LaunchConfiguration('flight_rate').perform(context)

    if enable_test:
        try:
            test_num = int(test_number)
            if test_num <= 0:
                raise ValueError("test_number must be > 0.")
        except ValueError as e:
            raise RuntimeError(f"Invalid test_number '{test_number}': must be positive integer.") from e

        drones_file_path = os.path.join(
            '/workspaces/swarm_of_drons/tests',
            f'test{test_num}',
            'drones.csv'
        )
    else:
        drones_file_path = default_file

    return [
        Node(
            package='drones_visualization',
            executable='visualization',
            name='visualization1',
            parameters=[{
                'use_sim_time': True,
                'drones_file': drones_file_path,
                'time_scale': float(time_scale),
                'flight_rate': float(flight_rate)
            }],
        )
    ]
