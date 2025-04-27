from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    drones_file = DeclareLaunchArgument(
        'drones_file', 
        default_value=TextSubstitution(
            text='/workspaces/swarm_of_drons/tests/test1/drones.csv'
        )
    )

    controller1_node = Node(
        package='drone_controller',
        name='drone_controller1',
        executable='drone_controller',
        parameters = [{
            'use_sim_time': True,
            'drones_file': LaunchConfiguration('drones_file'),
        }],
    )


    return LaunchDescription([
        drones_file,
        controller1_node
    ])