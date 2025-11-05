#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 22 16:08:56 2025

@author: disa
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mov6',
            executable='jetbot_draw',
            name='jetbot_draw',
            output='screen'
        ),
        Node(
            package='mov6',
            executable='jetbot_service',
            name='jetbot_service',
            output='screen'
        )
    ])