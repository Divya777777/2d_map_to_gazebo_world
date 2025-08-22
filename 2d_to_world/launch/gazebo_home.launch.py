import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package paths
    pkg_navigation_demo = get_package_share_directory('navigation_demo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file path
    world_file = os.path.join(pkg_navigation_demo, 'worlds', 'home_environment.world')

    # URDF file path
    urdf_file = os.path.join(pkg_navigation_demo, 'urdf', 'diff_bot.urdf.xacro')

    # Process the XACRO file
    robot_desc = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_file
    ])

    # Debug output
    debug_actions = [
        LogInfo(msg=f"Package path: {pkg_navigation_demo}"),
        LogInfo(msg=f"URDF path: {urdf_file}"),
        ExecuteProcess(
            cmd=['ls', '-la', os.path.join(pkg_navigation_demo, 'urdf')],
            output='screen'
        )
    ]

    # Gazebo Harmonic launch
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-r', world_file],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    # Spawn Robot - Using ros_gz_sim's spawner
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Delay spawn after Gazebo is ready
    delayed_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_sim,
            on_exit=[spawn_entity]
        )
    )

    return LaunchDescription(debug_actions + [
        gz_sim,
        robot_state_publisher,
        # delayed_spawn
    ])