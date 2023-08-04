#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package = 'ur5_code',
            executable = 'ur5_ft_sensor',
        ),

        Node(
            package = 'ur5_code',
            executable='ur5_ft300_data_filter',
        )

    ])