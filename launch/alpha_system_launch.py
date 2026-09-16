from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('lunar_simulator')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    world_arg = DeclareLaunchArgument('world', default_value='lunar_surface.sdf')

    gz_args = [
        PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')]),
        ' -v4',
    ]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return LaunchDescription([
        world_arg,
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                pkg_share, '/worlds:',
                pkg_share, '/alpha_model',
            ],
        ),
        gz_sim,
        Node(
            package='lunar_simulator',
            executable='alpha_node',
            name='alpha_system_node',
            parameters=[{'publish_rate_hz': 20.0}],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution(
                    [pkg_share, 'config', 'ros_gz_bridge.yaml']
                )
            }],
        ),
    ])
