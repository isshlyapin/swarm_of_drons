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
    drones_file_arg = DeclareLaunchArgument(
        'drones_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test2/drones.csv'
        ),
        description='Path to CSV file with drone parameters.'
    )

    return launch.LaunchDescription([
        drones_file_arg,
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
    drones_file_config = LaunchConfiguration('drones_file')
    drones = read_drones_file(context, drones_file_config)

    nodes = []

    # Drone controller
    nodes.append(
        ComposableNode(
            package='drone_composition',
            plugin='DroneComposition::DroneController',
            name='drone_controller',
            parameters=[{
                'use_sim_time': True,
                'drones_file': drones_file_config,
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


