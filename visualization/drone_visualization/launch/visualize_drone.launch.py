#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('drone_visualization'),
        'urdf',
        'simple_drone.urdf'
    )
    
    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(
        get_package_share_directory('drone_visualization'),
        'rviz',
        'drone_view.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node
    ])
