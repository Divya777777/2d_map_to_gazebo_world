from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('output_world', default_value=''),
        DeclareLaunchArgument('wall_height', default_value='2.0'),
        DeclareLaunchArgument('wall_thickness', default_value='0.1'),
        
        Node(
            package='navigation_demo',
            executable='2d_to_world',
            name='map_to_gazebo',
            output='screen',
            parameters=[{
                'map_path': LaunchConfiguration('map_path'),
                'output_world': LaunchConfiguration('output_world'),
                'wall_height': LaunchConfiguration('wall_height'),
                'wall_thickness': LaunchConfiguration('wall_thickness')
            }]
        )
    ])