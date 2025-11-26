from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    pkg_launch = os.path.join(os.getenv('HOME'), 'temp', 'g06_prii3_ws', 'src', 'g06_prii3_nav_turtlebot', 'launch')
    map_file = os.path.join(os.getenv('HOME'), 'mapaclase.yaml')
    nav2_launch_file = '/opt/ros/foxy/share/turtlebot3_navigation2/launch/navigation2.launch.py'
    rviz_file = '/opt/ros/foxy/share/turtlebot3_navigation2/rviz/nav2_default_view.rviz'

    return LaunchDescription([

        # Tu mundo propio en Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_launch, 'f1l3_world_launch.py'))
        ),

        # Nav2 oficial SIN RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': 'True',
                'map': map_file,
                'use_rviz': 'False'  # Evita que Nav2 abra su propio RViz
            }.items()
        ),

        # UN SOLO RViz con el mapa
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # Script de navegación con detección de ArUco
        Node(
            package='g06_prii3_nav_turtlebot',
            executable='cositas_aruco',
            name='aruco_waypoints_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])