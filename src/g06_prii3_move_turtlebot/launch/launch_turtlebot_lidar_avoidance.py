from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g06_prii3_move_turtlebot',
            executable='turtlebot_draw_lidar_avoidance',
            name='turtlebot_draw_lidar_avoidance',
            output='screen'
        ),
        Node(
            package='g06_prii3_move_turtlebot',
            executable='turtlebot_service',
            name='turtlebot_service',
            output='screen'
        )
    ])