#!/usr/bin/env python3

'''
@File:   kalman_filter_launch.py

@Brief:  Launch the complete Alpha localisation estimation stack.

@Date:   15/09/2026

'''

# Regular library imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Custom library Imports
# None


def generate_launch_description():
    """Build the four-node localisation launch description."""
    packageShare = FindPackageShare('lunar_simulator')
    parameterFile = PathJoinSubstitution(
        [packageShare, 'config', 'localisation.yaml']
    )
    useSimTime = LaunchConfiguration('use_sim_time')

    executables = [
        'inertial_odometry_node',
        'visual_odometry_node',
        'wheel_odometry_node',
        'kalman_filter_node',
    ]
    nodes = [
        Node(
            package='lunar_simulator',
            executable=executable,
            output='screen',
            parameters=[parameterFile, {'use_sim_time': useSimTime}],
        )
        for executable in executables
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *nodes,
    ])
