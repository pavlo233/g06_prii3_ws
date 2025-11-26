from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path 
def generate_launch_description():

    pkg_dir = str(Path(__file__).parent)

    # --- Rutas Locales (Modificadas para ser relativas a pkg_dir) ---
    # f1l3_world_launch.py está en la misma carpeta
    gazebo_launch_file = os.path.join(pkg_dir, 'f1l3_world_launch.py')
    # mapaclase.yaml está en la misma carpeta
    map_file = os.path.join(pkg_dir, 'mapaclase.yaml')
    nav2_launch_file = '/opt/ros/foxy/share/turtlebot3_navigation2/launch/navigation2.launch.py'
    rviz_file = '/opt/ros/foxy/share/turtlebot3_navigation2/rviz/nav2_default_view.rviz'

    return LaunchDescription([

        # Tu mundo propio en Gazebo (Usa la ruta local)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),

        # Nav2 oficial SIN RViz (Usa la ruta absoluta)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': 'True',
                'map': map_file,  # El mapa sí usa la ruta local
                'use_rviz': 'False'
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