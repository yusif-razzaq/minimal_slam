#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('minimal_slam')
    
    # Process URDF
    urdf_file = os.path.join(pkg, 'urdf', 'ackermann_car.xacro')
    robot_desc = xacro.process_file(urdf_file).toxml()
    
    # Gazebo with room world
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
    
    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )
    
    # Joint state publisher (publishes TF for passive joints like front wheels)
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    
    # Spawn robot
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'car']
    )
    
    # SLAM Toolbox
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
    
    # RViz (no custom config - will use defaults)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_pub,
        joint_state_pub,
        spawn,
        slam,
        rviz
    ])

