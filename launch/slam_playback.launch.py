#!/usr/bin/env python3
"""
SLAM with Recorded Rosbag Data

This launch file plays back recorded data and runs SLAM.
No simulation - uses recorded sensor data from your robot.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('minimal_slam')
    
    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join(pkg, '..', '..', '..', 'robot_data'),
        description='Path to the rosbag directory'
    )
    
    # Process URDF
    urdf_file = os.path.join(pkg, 'urdf', 'ackermann_car.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # ========================================
    # 1. Robot State Publisher (for TF frames)
    # ========================================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  # Important: use bag timestamp
        }]
    )
    
    # ========================================
    # 2. SLAM Toolbox
    # ========================================
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam.yaml'),
            'use_sim_time': 'true'  # Critical: sync with bag playback
        }.items()
    )
    
    # ========================================
    # 3. RViz
    # ========================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        bag_path_arg,
        robot_state_pub,
        slam,
        rviz
    ])

