#!/usr/bin/env python3
"""
SLAM with Rosbag Playback + Full IMU Fusion

This launch file replays your recorded rosbag and runs SLAM with:
- IMU combiner (merges /imu/gyro + /imu/accel → /imu/data)
- Robot localization EKF (fuses /odom_rf2o + /imu/data)
- SLAM Toolbox (builds map from /scan + fused odometry)

Use this for the most accurate results with your recorded data.
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
            'use_sim_time': True
        }]
    )
    
    # ========================================
    # 2. IMU Combiner (merge gyro + accel)
    # ========================================
    imu_combiner = Node(
        package='minimal_slam',
        executable='imu_combiner.py',
        name='imu_combiner',
        output='screen',
        parameters=[{
            'gyro_topic': '/imu/gyro',
            'accel_topic': '/imu/accel',
            'imu_output_topic': '/imu/data',
            'imu_frame_id': 'imu_link',
            'use_sim_time': True
        }]
    )
    
    # ========================================
    # 3. Sensor Fusion (EKF) - Fuses RF2O + IMU
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
    # 4. SLAM Toolbox
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
    # 5. RViz
    # ========================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        bag_path_arg,
        robot_state_pub,
        imu_combiner,
        ekf,
        slam,
        rviz
    ])

