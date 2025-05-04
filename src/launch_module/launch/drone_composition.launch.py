import launch
import csv
import os
from typing import List

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generates the full launch description."""
    enable_test_arg = DeclareLaunchArgument(
        'enable_test',
        default_value=TextSubstitution(text='false'),
        description='Enable test mode.'
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
        description='Path to CSV file with drone parameters.'
    )

    flight_rate_arg = DeclareLaunchArgument(
        'flight_rate',
        default_value=TextSubstitution(text='50.0'),
        description='Rate at which the drones will fly.'
    )

    time_scale_arg = DeclareLaunchArgument(
        'time_scale',
        default_value=TextSubstitution(text='1.0'),
        description='Time scale for the simulation.'
    )


    return launch.LaunchDescription([
        enable_test_arg,
        test_number_arg,
        drones_file_arg,
        flight_rate_arg,
        time_scale_arg,
        OpaqueFunction(function=create_nodes)
    ])

def read_drones_file(context, file_path: LaunchConfiguration) -> List[dict]:
    """Reads the CSV file with drone parameters."""
    path = file_path.perform(context)
    drones = []
    
    if not os.path.exists(path):
        raise RuntimeError(f"Drones file not found: {path}")
    
    with open(path, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            drones.append(row)
    
    return drones


def create_nodes(context):
    """Creates the list of composable node descriptions."""
    enable_test = LaunchConfiguration('enable_test').perform(context).lower() == 'true'
    test_number = LaunchConfiguration('test_number').perform(context)
    drones_file_config = LaunchConfiguration('drones_file')
    flight_rate_config = LaunchConfiguration('flight_rate')
    time_scale_config = LaunchConfiguration('time_scale')

    # Выбор пути к CSV файлу в зависимости от режима
    if enable_test:
        try:
            test_num_int = int(test_number)
            if test_num_int <= 0:
                raise ValueError("Test number must be > 0.")
        except ValueError as e:
            raise RuntimeError(f"Invalid test_number: {test_number}. Must be a positive integer.") from e

        test_path = os.path.join(
            '/workspaces/swarm_of_drons/tests',
            f'test{test_num_int}',
            'drones.csv'
        )
    else:
        test_path = drones_file_config.perform(context)

    drones = read_drones_file(context, TextSubstitution(text=test_path))

    nodes = []

    # Drone controller
    nodes.append(
        ComposableNode(
            package='drone_composition',
            plugin='DroneComposition::DroneController',
            name='drone_controller',
            parameters=[{
                'use_sim_time': True,
                'drones_file': test_path,  # строка, а не LaunchConfiguration
            }]
        )
    )

    # Individual drone nodes
    for drone in drones:
        drone_name = f"{drone['model']}_{drone['id']}"
        node = ComposableNode(
            package='drone_composition',
            plugin='DroneComposition::Drone',
            name=drone_name,
            parameters=[{
                'use_sim_time': True,
                'flight_rate': float(flight_rate_config.perform(context)),
                'time_scale': float(time_scale_config.perform(context)),
                'pose_x': float(drone['x']),
                'pose_y': float(drone['y']),
                'pose_z': float(drone['z']),
            }]
        )
        nodes.append(node)

    container = ComposableNodeContainer(
        name='drone_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )

    return [container]

