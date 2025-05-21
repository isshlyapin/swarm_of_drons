from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description(): 
    controller1_node = Node(
        package='navigator',
        name='navigator',
        executable='navigator',
        parameters = [{
            'use_sim_time': True
        }],
    )


    return LaunchDescription([
        controller1_node
    ])