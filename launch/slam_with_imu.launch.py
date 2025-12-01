#!/usr/bin/env python3
"""
SLAM with LiDAR + IMU Fusion (No wheel encoders needed!)

This launch file implements the full pipeline:
  LiDAR → rf2o (scan matching) → odometry estimates
  IMU → orientation and angular velocity
  Both → robot_localization (EKF) → fused odometry
  Fused odometry → SLAM Toolbox → map

For physical robots without wheel encoders.
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
    # 1. Gazebo (for testing - remove for real robot)
    # ========================================
    world_file = os.path.join(
        get_package_share_directory('my_bot'),
        'worlds',
        'room.world'
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    # ========================================
    # 2. Robot State Publisher
    # ========================================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    
    # ========================================
    # 3. Spawn Robot
    # ========================================
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'car']
    )
    
    # ========================================
    # 4. LiDAR Odometry (rf2o) - Replaces wheel encoders!
    # ========================================
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,  # EKF will publish the final TF
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'verbose': False
        }]
    )
    
    # ========================================
    # 5. Sensor Fusion (EKF) - Fuses LiDAR odometry + IMU
    # ========================================
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[
            ('/odometry/filtered', '/odom')  # Output fused odometry as /odom
        ]
    )
    
    # ========================================
    # 6. SLAM Toolbox
    # ========================================
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam.yaml'),
            'use_sim_time': 'true'
        }.items()
    )
    
    # ========================================
    # 7. RViz
    # ========================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        # Simulation (remove for real robot)
        gazebo,
        robot_state_pub,
        joint_state_pub,
        spawn,
        
        # LiDAR odometry + IMU fusion
        rf2o,
        ekf,
        
        # SLAM
        slam,
        
        # Visualization
        rviz
    ])

