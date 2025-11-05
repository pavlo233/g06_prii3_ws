#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Nodo de dibujo con LIDAR
        Node(
            package='mov6',
            executable='jetbot_draw_lidar',
            name='jetbot_draw_lidar',
            output='screen',
        ),

        # Nodo del servicio
        Node(
            package='mov6',
            executable='jetbot_service',
            name='jetbot_service',
            output='screen',
        ),
    ])


