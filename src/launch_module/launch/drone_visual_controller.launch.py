from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drones_visualization',
            name='visualization1',
            executable='visualization',
            parameters = [{
                'use_sim_time': True
            }],
        )
    ])