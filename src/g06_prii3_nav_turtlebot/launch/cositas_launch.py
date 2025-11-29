from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path 

def generate_launch_description():
    # Obtener el directorio donde se encuentra este archivo (la carpeta actual)
    pkg_dir = str(Path(__file__).parent)

    # --- Rutas Locales (Modificadas para ser relativas a pkg_dir) ---
    # f1l3_world_launch.py está en la misma carpeta
    gazebo_launch_file = os.path.join(pkg_dir, 'f1l3_world_launch.py')
    # mapaclase.yaml está en la misma carpeta
    map_file = os.path.join(pkg_dir, 'mapaclase.yaml')
    
    # --- Rutas Oficiales (Mantenidas sin cambios, usando rutas absolutas) ---
    nav2_launch_file = '/opt/ros/foxy/share/turtlebot3_navigation2/launch/navigation2.launch.py'
    rviz_file = '/opt/ros/foxy/share/turtlebot3_navigation2/rviz/nav2_default_view.rviz'

    # Lanzar Gazebo primero (inmediatamente) puto gazebo de mierda
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # Esperar 5 segundos y lanzar rviz
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'use_sim_time': 'True',
                    'map': map_file,
                    'use_rviz': 'False'
                }.items()
            )
        ]
    )


    # Esperar 12 segundos y lanzar el script de navegación
    script_launch = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='g06_prii3_nav_turtlebot',
                executable='cositas',
                name='waypoints_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        
        script_launch
    ])