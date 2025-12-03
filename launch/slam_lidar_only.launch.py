#!/usr/bin/env python3
"""
SLAM with LiDAR ONLY - No IMU, No Wheel Encoders

This launch file uses ONLY laser scan data to:
1. Generate odometry from laser scans (RF2O)
2. Build a map using SLAM Toolbox

Perfect for testing with rosbag data that only has /scan topic.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('minimal_slam')
    
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
            'use_sim_time': True  # Critical for rosbag playback
        }]
    )
    
    # ========================================
    # 2. Static TF: odom -> base_footprint
    # ========================================
    # Publish initial transform (RF2O will update this)
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )
    
    # ========================================
    # 3. LiDAR Odometry (RF2O) - Generates odometry from laser scans
    # ========================================
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,  # RF2O publishes odom->base_footprint transform
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,  # Update frequency
            'verbose': True,  # Show status messages
            'use_sim_time': True
        }]
    )
    
    # ========================================
    # 4. SLAM Toolbox
    # ========================================
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam.yaml'),
            'use_sim_time': 'true'  # Critical for rosbag playback
        }.items()
    )
    
    # ========================================
    # 5. RViz
    # ========================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        robot_state_pub,
        static_tf_pub,
        rf2o,
        slam,
        rviz
    ])

