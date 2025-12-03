#!/usr/bin/env python3
"""
IMU Combiner Node

Combines separate /imu/gyro and /imu/accel topics (Vector3)
into a single /imu/data topic (sensor_msgs/Imu).

This is needed because your rosbag has split IMU data,
but robot_localization EKF expects combined sensor_msgs/Imu format.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ImuCombiner(Node):
    def __init__(self):
        super().__init__('imu_combiner')
        
        # Declare parameters
        self.declare_parameter('gyro_topic', '/imu/gyro')
        self.declare_parameter('accel_topic', '/imu/accel')
        self.declare_parameter('imu_output_topic', '/imu/data')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('use_sim_time', True)
        
        # Get parameters
        gyro_topic = self.get_parameter('gyro_topic').value
        accel_topic = self.get_parameter('accel_topic').value
        imu_output_topic = self.get_parameter('imu_output_topic').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        
        # QoS for sensor data (matches your rosbag: BEST_EFFORT)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscribers
        self.gyro_sub = self.create_subscription(
            Vector3, 
            gyro_topic, 
            self.gyro_callback, 
            qos
        )
        self.accel_sub = self.create_subscription(
            Vector3, 
            accel_topic, 
            self.accel_callback, 
            qos
        )
        
        # Publisher
        self.imu_pub = self.create_publisher(
            Imu, 
            imu_output_topic, 
            qos
        )
        
        # Storage for latest messages
        self.latest_gyro = None
        self.latest_accel = None
        self.gyro_time = None
        self.accel_time = None
        
        # Stats
        self.gyro_count = 0
        self.accel_count = 0
        self.imu_count = 0
        
        # Timer for status logging
        self.create_timer(5.0, self.log_status)
        
        self.get_logger().info(f'IMU Combiner started')
        self.get_logger().info(f'  Subscribing to: {gyro_topic}, {accel_topic}')
        self.get_logger().info(f'  Publishing to: {imu_output_topic}')
    
    def gyro_callback(self, msg):
        """Callback for gyroscope data"""
        self.latest_gyro = msg
        self.gyro_time = self.get_clock().now()
        self.gyro_count += 1
        self.publish_combined()
    
    def accel_callback(self, msg):
        """Callback for accelerometer data"""
        self.latest_accel = msg
        self.accel_time = self.get_clock().now()
        self.accel_count += 1
        self.publish_combined()
    
    def publish_combined(self):
        """Publish combined IMU message when both sensors have data"""
        if self.latest_gyro is None or self.latest_accel is None:
            return
        
        # Create combined IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id
        
        # Angular velocity (from gyroscope) - rad/s
        imu_msg.angular_velocity.x = self.latest_gyro.x
        imu_msg.angular_velocity.y = self.latest_gyro.y
        imu_msg.angular_velocity.z = self.latest_gyro.z
        
        # Covariance for angular velocity (adjust based on your IMU specs)
        # For now, using reasonable defaults for consumer-grade IMU
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        
        # Linear acceleration (from accelerometer) - m/s^2
        imu_msg.linear_acceleration.x = self.latest_accel.x
        imu_msg.linear_acceleration.y = self.latest_accel.y
        imu_msg.linear_acceleration.z = self.latest_accel.z
        
        # Covariance for linear acceleration
        imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]
        
        # Orientation (not computed from raw gyro/accel)
        # Set to identity quaternion and mark as unknown with -1 covariance
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]
        
        # Publish
        self.imu_pub.publish(imu_msg)
        self.imu_count += 1
    
    def log_status(self):
        """Periodic status logging"""
        self.get_logger().info(
            f'Status: Gyro={self.gyro_count}, '
            f'Accel={self.accel_count}, '
            f'Combined IMU={self.imu_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuCombiner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down IMU Combiner')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

