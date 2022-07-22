"""Path Planning Launch File module."""
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    """
    Launch the Path Planning package for Testing.

    Generates the launch description and launches the path planning package with all its nodes:
    Path Planner, Optimization Service, Cone Publisher and Planned Trajectory Subscriber.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'track_name', default_value=TextSubstitution(text='small_track.csv')),
        DeclareLaunchArgument(
            'laps', default_value=TextSubstitution(text='2')),
        DeclareLaunchArgument(
            'timer_period', default_value=TextSubstitution(text='0.2')),
        DeclareLaunchArgument(
            'optimization_type', default_value=TextSubstitution(text='mincurv')),
        DeclareLaunchArgument(
            'minimum_track_width', default_value=TextSubstitution(text='None')),
        DeclareLaunchArgument(
            'show_calc_times', default_value=TextSubstitution(text='True')),
        DeclareLaunchArgument(
            'show_plot_exploration', default_value=TextSubstitution(text='True')),
        DeclareLaunchArgument(
            'show_plot_optimization', default_value=TextSubstitution(text='False')),
        DeclareLaunchArgument(
            'mock_current_position', default_value=TextSubstitution(text='True')),
        Node(
            package='path_planning',
            executable='optimization_service',
            parameters=[{
                'optimization_type': LaunchConfiguration('optimization_type'),
                'minimum_track_width': LaunchConfiguration('minimum_track_width'),
                'show_plot_optimization': LaunchConfiguration('show_plot_optimization'),
            }]
        ),
        Node(
            package='path_planning',
            executable='path_planner',
            parameters=[{
                'track_name': LaunchConfiguration('track_name'),
                'laps': LaunchConfiguration('laps'),
                'show_calc_times': LaunchConfiguration('show_calc_times'),
                'show_plot_exploration': LaunchConfiguration('show_plot_exploration'),
                'mock_current_position': LaunchConfiguration('mock_current_position')
            }]
        ),
        Node(
            package='path_planning',
            executable='cone_publisher',
            parameters=[{
                'track_name': LaunchConfiguration('track_name'),
                'laps': LaunchConfiguration('laps'),
                'timer_period': LaunchConfiguration('timer_period')
            }]
        ),
        Node(
            package='path_planning',
            executable='planned_trajectory_subscriber'
        )
    ])
