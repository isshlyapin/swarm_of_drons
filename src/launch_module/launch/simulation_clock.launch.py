from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    start_paused_arg = DeclareLaunchArgument(
        'start_paused', default_value=TextSubstitution(text='False')
    )
    time_scale_arg = DeclareLaunchArgument(
        'time_scale', default_value=TextSubstitution(text='1.0')
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value=TextSubstitution(text='100')
    )

    return LaunchDescription([
        start_paused_arg,
        time_scale_arg,
        publish_rate_arg,
        Node(
            package='clock_simulation',
            executable='sim_clock',
            name='sim_clock',
            parameters=[{
                'time_scale':   LaunchConfiguration('time_scale'),
                'start_paused': LaunchConfiguration('start_paused'),
                'publish_rate': LaunchConfiguration('publish_rate')
            }]
        )
    ])