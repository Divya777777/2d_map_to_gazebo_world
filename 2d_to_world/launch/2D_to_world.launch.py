from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_demo',
            executable='2d_to_world',
            name='map_to_gazebo',
            output='screen',
            parameters=[{
                'map_path': '/home/divya/project/src/navigation_demo/maps/office_2.yaml',
                'output_world': '/home/divya/maps/genrated.world',
                'wall_height': 2.0,
                'wall_thickness': 0.1
            }]
        )
    ])