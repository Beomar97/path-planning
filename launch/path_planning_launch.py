from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'track_name', default_value=TextSubstitution(text='sim_tool')),
        DeclareLaunchArgument(
            'laps', default_value=TextSubstitution(text='2')),
        DeclareLaunchArgument(
            'optimization_type', default_value=TextSubstitution(text='mincurv')),
        DeclareLaunchArgument(
            'show_plot_exploration', default_value=TextSubstitution(text='False')),
        DeclareLaunchArgument(
            'show_plot_optimization', default_value=TextSubstitution(text='False')),
        DeclareLaunchArgument(
            'mock_current_position', default_value=TextSubstitution(text='True')),
        Node(
            package='path_planning',
            namespace='path_planning',
            executable='optimization_service',
            name='optimization_service'
        ),
        Node(
            package='path_planning',
            namespace='path_planning',
            executable='path_planner',
            name='path_planner',
            parameters=[{
                'track_name': LaunchConfiguration('track_name'),
                'laps': LaunchConfiguration('laps'),
                'optimization_type': LaunchConfiguration('optimization_type'),
                'show_plot_exploration': LaunchConfiguration('show_plot_exploration'),
                'show_plot_optimization': LaunchConfiguration('show_plot_optimization'),
                'mock_current_position': LaunchConfiguration('mock_current_position')
            }]
        ),
        Node(
            package='path_planning',
            namespace='path_planning',
            executable='cone_publisher',
            name='cone_publisher',
            parameters=[{
                'track_name': LaunchConfiguration('track_name'),
                'laps': LaunchConfiguration('laps')
            }]
        )
    ])
