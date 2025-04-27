from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    missions_file = DeclareLaunchArgument(
        'mfile', 
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test2/missions.json'
        )
    )

    publisher_node = Node(
        package='mission_publisher',
        executable='mission_publisher',
        name='mission_publisher',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=['-f', LaunchConfiguration('mfile'), '--all']
    )

    return LaunchDescription([
        missions_file,
        publisher_node
    ])