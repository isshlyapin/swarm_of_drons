from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test2/map.csv'
        )
    )

    map_node = Node(
        package='map_visual',
        executable='map_publisher_node',
        parameters=[{
            'use_sime_time': True,
            'map_file': LaunchConfiguration('map_file')
        }]
    )

    return LaunchDescription([
        map_file_arg,
        map_node
    ])