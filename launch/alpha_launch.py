"""Alpha system launch file.

Source of truth for which nodes the Alpha system owns. Launches the
single ``alpha_node`` executable (which composes ``alpha_driver_node``,
``ground_truth``, ``inertial_odometry``, ``visual_odometry``,
``wheel_odometry``, ``continuous_ekf``, ``ackermann_controller`` and
``startup_supervisor`` in one process) plus, optionally, Gazebo, the ``ros_gz_bridge`` and RViz.

``system_name`` (default ``alpha``) selects the topic namespace every node
uses for its own default topic names (e.g.
``/alpha/localisation/kalman_filter/odometry``). It is passed to the nodes
only for those topic defaults; all file paths below are hardcoded to the
Alpha system's own ``parameters/systems/alpha/`` tree, ``config/`` entry
and ``alpha`` model/RViz assets, so overriding ``system_name`` reuses the
same executable under a different topic namespace without loading a
different parameter tree. Adding a system means a new ``src/systems/<name>/``
plus a new ``launch/<name>_launch.py`` following this file's pattern, never
an edit to ``scripts/launch_simulator.sh``.

Standalone use (starts Gazebo and the bridge itself)::

    ros2 launch space_robotics_simulator alpha_launch.py

``scripts/launch_simulator.sh`` instead handles Gazebo spawn/pause/unpause
itself and delegates only node startup here with ``launch_gazebo:=false
launch_bridge:=false``.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('space_robotics_simulator')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    system_name_arg = DeclareLaunchArgument(
        'system_name',
        default_value='alpha',
        description='Topic namespace every Alpha node builds its defaults from.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Follow simulation time on the alpha_node process.',
    )
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='73'),
        description='DDS domain used to isolate this simulator and its /clock.',
    )
    gz_partition_arg = DeclareLaunchArgument(
        'gz_partition',
        default_value=EnvironmentVariable(
            'GZ_PARTITION',
            default_value=[
                'space_robotics_simulator_',
                EnvironmentVariable('ROS_DOMAIN_ID', default_value='73'),
            ],
        ),
        description='Gazebo Transport partition isolating the simulator clock.',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='crater_field.sdf',
        description='World file under share/space_robotics_simulator/environment/lunar/crater_field/.',
    )
    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Start Gazebo here; scripts/launch_simulator.sh sets false.',
    )
    launch_bridge_arg = DeclareLaunchArgument(
        'launch_bridge',
        default_value='true',
        description='Start the ros_gz bridge here; scripts/launch_simulator.sh sets false.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Open RViz with src/systems/alpha/alpha.rviz.',
    )
    # Console rule: a short fixed prefix and no 20-digit timestamp, so
    # each wrapped node line carries at most 40 characters of text after
    # it. rcl uses the same format for the node's file log; per-line wall
    # timestamps remain in the launch log, which prefixes every line.
    console_format_arg = DeclareLaunchArgument(
        'console_format',
        default_value='[{severity}] [{name}]: {message}',
        description='RCUTILS_CONSOLE_OUTPUT_FORMAT for alpha_node.',
    )

    # Hardcoded Alpha parameter tree: 9 files. use_sim_time is wired as a
    # Node parameter below, and the bridge is exempt per D5.
    parameters_root = [pkg_share, 'parameters', 'systems', 'alpha']
    alpha_parameters = [
        PathJoinSubstitution(parameters_root + ['alpha_drivers', 'alpha_drivers.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_localisation', 'alpha_kalman_filter.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_localisation', 'ground_truth.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_localisation', 'inertial_odometry.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_localisation', 'visual_odometry.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_localisation', 'wheel_odometry.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_control', 'ackermann_controller.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_control', 'estimate_low_pass_filter.yaml']),
        PathJoinSubstitution(parameters_root + ['alpha_supervisor', 'startup_supervisor.yaml']),
    ]

    gz_args = [
        PathJoinSubstitution(
            [pkg_share, 'environment', 'lunar', 'crater_field', LaunchConfiguration('world')]
        ),
        ' -v4',
    ]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
    )

    alpha_node = Node(
        package='space_robotics_simulator',
        executable='alpha_node',
        parameters=alpha_parameters + [
            {'system_name': LaunchConfiguration('system_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        # Print each node line as-is: the "[alpha_node-1] " process prefix
        # would otherwise push every line past the console width.
        output_format='{line}',
        additional_env={
            'RCUTILS_CONSOLE_OUTPUT_FORMAT': LaunchConfiguration('console_format'),
        },
    )

    # The bridge reads share/.../config/alpha/ros_gz_bridge.yaml
    # (DIRECTORY config installs to share/.../config/). It is not an
    # rclcpp time-user, so no use_sim_time here.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [pkg_share, 'config/alpha/ros_gz_bridge.yaml']
            ),
        }],
        condition=IfCondition(LaunchConfiguration('launch_bridge')),
    )

    # RViz output goes to the launch log only, keeping the terminal for
    # project node lines; scripts/launch_simulator.sh starts its own RViz.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'alpha', 'alpha.rviz'])],
        output='log',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        system_name_arg,
        use_sim_time_arg,
        ros_domain_id_arg,
        gz_partition_arg,
        world_arg,
        launch_gazebo_arg,
        launch_bridge_arg,
        launch_rviz_arg,
        console_format_arg,
        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID',
            value=LaunchConfiguration('ros_domain_id'),
        ),
        SetEnvironmentVariable(
            name='GZ_PARTITION',
            value=LaunchConfiguration('gz_partition'),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                PathJoinSubstitution(
                    [pkg_share, 'environment', 'lunar', 'crater_field']
                ),
                ':',
                PathJoinSubstitution(
                    [pkg_share, 'environment', 'martian', 'plain_stub']
                ),
                ':',
                PathJoinSubstitution([pkg_share, 'alpha_model']),
            ],
        ),
        gz_sim,
        alpha_node,
        bridge,
        rviz,
    ])
