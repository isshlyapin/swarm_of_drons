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
        description='Enable test mode (use test{n}/missions.csv)'
    )

    test_number_arg = DeclareLaunchArgument(
        'test_number',
        default_value=TextSubstitution(text='0'),
        description='Test number (must be > 0)'
    )

    missions_file_arg = DeclareLaunchArgument(
        'missions_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test1/missions.csv'
        ),
        description='Path to missions.csv (used only if enable_test is false)'
    )

    graph_file_arg = DeclareLaunchArgument(
        'graph_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test1/graph.csv'
        ),
        description='Path to graph.csv (used only if enable_test is false)'
    )

    edges_file_arg = DeclareLaunchArgument(
        'edges_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test1/edges.csv'
        ),
        description='Path to edges.csv (used only if enable_test is false)'
    )

    time_scale_arg = DeclareLaunchArgument(
        'time_scale',
        default_value=TextSubstitution(text='1.0'),
        description='Time scale for simulation'
    )

    return LaunchDescription([
        enable_test_arg,
        test_number_arg,
        missions_file_arg,
        graph_file_arg,
        edges_file_arg,
        time_scale_arg,
        OpaqueFunction(function=launch_controller_node)
    ])

def launch_controller_node(context, *args, **kwargs):
    enable_test = LaunchConfiguration('enable_test').perform(context).lower() == 'true'
    test_number = LaunchConfiguration('test_number').perform(context)
    default_missions_file = LaunchConfiguration('missions_file').perform(context)
    default_graph_file = LaunchConfiguration('graph_file').perform(context)
    default_edges_file = LaunchConfiguration('edges_file').perform(context)
    time_scale = LaunchConfiguration('time_scale').perform(context)

    if enable_test:
        try:
            test_num = int(test_number)
            if test_num <= 0:
                raise RuntimeError("test_number must be > 0 when enable_test is true.")
        except ValueError:
            raise RuntimeError(f"Invalid test_number: {test_number}. Must be a positive integer.")

        missions_file_path = os.path.join(
            '/workspaces/swarm_of_drons/tests',
            f'test{test_num}',
            'missions.csv'
        )
        graph_file_path = os.path.join(
            '/workspaces/swarm_of_drons/tests',
            f'test{test_num}',
            'graph.csv'
        )
        edges_file_path = os.path.join(
            '/workspaces/swarm_of_drons/tests',
            f'test{test_num}',
            'edges.csv'
        )
    else:
        missions_file_path = default_missions_file
        graph_file_path = default_graph_file
        edges_file_path = default_edges_file


    node = Node(
        package='navigator2',
        executable='navigator2',
        name='navigator2',
        parameters=[{
            'use_sim_time': True,
            'missions_file': missions_file_path,
            'graph_file': graph_file_path,
            'edges_file': edges_file_path,
            'time_scale': float(time_scale)
        }]
    )

    return [node]
