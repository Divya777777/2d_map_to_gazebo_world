#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    TimerAction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Package and file paths
    package_name = 'navigation_demo'  # Your package name
    xacro_file = 'diff_bot.urdf.xacro'
    
    # Path to XACRO file
    xacro_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'urdf',
        xacro_file
    ])
    
    # Process XACRO to URDF
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_path]),
        value_type=str
    )
    
    # Launch Gazebo Harmonic with verbose output
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', 'empty.sdf'],
        output='screen',
        shell=False
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Spawn robot entity
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'diff_bot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.15'  # Higher spawn height
        ],
        output='screen'
    )
    
    # Bridge for ROS2-Gazebo communication (simplified)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',  # Comment out for now
            # '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',  # Comment out for now
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Delay spawning until Gazebo is fully loaded
    delayed_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_launch,
            on_start=[
                TimerAction(
                    period=5.0,  # Wait 5 seconds
                    actions=[spawn_robot]
                )
            ]
        )
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        delayed_spawn,
    ])