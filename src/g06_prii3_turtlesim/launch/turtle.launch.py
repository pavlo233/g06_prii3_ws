from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='g06_prii3_turtlesim',
            executable='turtle_draw',
            name='draw'
        ),
        Node(
            package='g06_prii3_turtlesim',
            executable='turtle_service',
            name='service'
        ),
    ])
