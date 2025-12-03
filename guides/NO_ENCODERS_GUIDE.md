# SLAM Without Wheel Encoders (LiDAR + IMU Only)

## Yes, It's Possible!

You can do SLAM with **only LiDAR and IMU** - no wheel encoders needed. The approach is called **scan-matching SLAM** or **LiDAR odometry**.

---

## How It Works

### Traditional SLAM (with encoders):
```
Wheel Encoders → Odometry → SLAM → Map
     +
  LiDAR
```

### SLAM Without Encoders (LiDAR + IMU):
```
LiDAR Scans → Scan Matching → Estimated Odometry → SLAM → Map
     +
   IMU (orientation)
```

**Key difference:** Instead of wheel encoders telling you how far you moved, the algorithm **compares consecutive laser scans** to estimate motion.

---

## The Pipeline

### 1. **LiDAR Scan Matching (Primary Motion Estimation)**

**What it does:**
- Compare current laser scan to previous scan
- Find how much the robot moved between scans
- This replaces wheel encoders!

**How it works:**
```
Scan at t=0:  |||||||||||  (robot sees walls)
               ↓ robot moves right
Scan at t=1:    |||||||||||  (walls shifted left in view)
               
Algorithm says: "Robot moved 0.3m right"
```

**Algorithms used:**
- ICP (Iterative Closest Point)
- Scan-to-scan matching
- Scan-to-map matching

### 2. **IMU (Orientation Assistance)**

**What it provides:**
- Angular velocity (how fast turning)
- Orientation (which way facing)
- Helps when robot rotates quickly

**Benefits:**
- Better rotation estimates
- Reduces scan-matching failures
- Smoother motion estimation

### 3. **SLAM Toolbox Integration**

**Good news:** `slam_toolbox` has built-in scan matching! You don't need separate odometry.

---

## Two Approaches

### Approach 1: Pure Scan-Matching (LiDAR Only)

**Pros:**
- Simplest setup
- Only need LiDAR
- slam_toolbox handles everything

**Cons:**
- Less accurate than with encoders
- Can fail in featureless environments
- Slower update rate

**Configuration:**
```yaml
slam_toolbox:
  ros__parameters:
    # NO odometry topic specified!
    scan_topic: /scan
    base_frame: base_link
    odom_frame: odom
    map_frame: map
    
    # Important: Enable strong scan matching
    use_scan_matching: true
    use_scan_barycenter: true
    
    # Slower updates for pure scan matching
    minimum_travel_distance: 0.1  # Update every 10cm
    minimum_travel_heading: 0.1   # Update every 0.1 rad
    
    # Stronger scan matching parameters
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
```

**What you need to publish:**
1. `/scan` topic (LiDAR)
2. TF: `odom` → `base_link` (static or from scan matching)

---

### Approach 2: IMU + LiDAR (Recommended)

**Use `robot_localization` package to fuse IMU + scan-based odometry**

**Pros:**
- More accurate
- Better handling of fast rotation
- Smoother motion estimates
- Industry standard

**Cons:**
- More complex setup
- Need to tune sensor fusion

**Pipeline:**
```
LiDAR → rf2o_laser_odometry → Laser Odom
                                    ↓
                              robot_localization (EKF)
                                    ↓
                              Fused Odometry → SLAM
   IMU ────────────────────────────↑
```

---

## Implementation: LiDAR + IMU Fusion

### Step 1: Install Required Packages

```bash
sudo apt install ros-humble-rf2o-laser-odometry \
                 ros-humble-robot-localization
```

### Step 2: LiDAR Odometry (replaces wheel encoders)

**Package:** `rf2o_laser_odometry`  
**What it does:** Estimates motion from laser scans

**Launch:**
```bash
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom_rf2o \
  -p base_frame_id:=base_link
```

**Output:** `/odom_rf2o` - odometry from LiDAR only

### Step 3: IMU Integration

**Your IMU must publish:**
- Topic: `/imu/data`
- Type: `sensor_msgs/Imu`
- Rate: 50-100 Hz
- Data: Orientation + angular velocity

### Step 4: Sensor Fusion (robot_localization)

**Create:** `config/ekf.yaml`
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    
    two_d_mode: true  # For 2D SLAM
    
    # Input sources
    odom0: /odom_rf2o  # LiDAR odometry
    odom0_config: [true,  true,  false,  # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
    
    imu0: /imu/data    # IMU data
    imu0_config: [false, false, false,   # x, y, z
                  false, false, true,    # roll, pitch, yaw (use yaw)
                  false, false, false,   # vx, vy, vz
                  false, false, true,    # vroll, vpitch, vyaw
                  false, false, false]   # ax, ay, az
    
    # Fused output
    odom_topic: /odometry/filtered
    publish_tf: true
```

**Launch:**
```bash
ros2 launch robot_localization ekf.launch.py \
  params_file:=config/ekf.yaml
```

**Output:** `/odometry/filtered` - fused IMU + LiDAR odometry

### Step 5: SLAM with Fused Odometry

**Update SLAM config:**
```yaml
slam_toolbox:
  ros__parameters:
    scan_topic: /scan
    odom_frame: odom
    base_frame: base_link
    
    # Now we have odometry!
    use_scan_matching: true  # Still helpful
```

**Launch SLAM:**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=config/slam.yaml \
  use_sim_time:=false
```

---

## Complete Launch File (LiDAR + IMU)

Create: `launch/slam_no_encoders.launch.py`

```python
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('minimal_slam')
    
    # 1. LiDAR Odometry
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,  # EKF will publish TF
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'freq': 10.0
        }]
    )
    
    # 2. Sensor Fusion (IMU + LiDAR odometry)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')]
    )
    
    # 3. SLAM
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'slam.yaml')]
    )
    
    # 4. Robot State Publisher (static transforms)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )
    
    return LaunchDescription([
        robot_state_pub,
        rf2o,
        ekf,
        slam
    ])
```

---

## What You Need to Provide

### 1. LiDAR Data
```bash
# Topic: /scan
# Type: sensor_msgs/LaserScan
# Rate: 5-10 Hz
ros2 topic pub /scan sensor_msgs/LaserScan "..."
```

### 2. IMU Data
```bash
# Topic: /imu/data
# Type: sensor_msgs/Imu
# Rate: 50-100 Hz
ros2 topic pub /imu/data sensor_msgs/Imu "..."
```

### 3. URDF (Robot Description)
```xml
<robot name="my_robot">
  <link name="base_link"/>
  
  <link name="laser"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0.2 0 0.3"/>  <!-- LiDAR position -->
  </joint>
  
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05"/>  <!-- IMU position -->
  </joint>
</robot>
```

---

## Comparison: With vs Without Encoders

| Feature | With Encoders | Without Encoders (LiDAR+IMU) |
|---------|---------------|------------------------------|
| Accuracy | ★★★★★ | ★★★★☆ |
| Setup Complexity | ★★☆☆☆ | ★★★★☆ |
| Cost | $$ (encoders) | $ (just IMU) |
| Works in hallways | ★★★★★ | ★★★☆☆ (needs features) |
| Works outdoors | ★★★★★ | ★★☆☆☆ (harder) |
| Rotation accuracy | ★★★★★ | ★★★★★ (with IMU) |
| Speed capability | Fast | Medium (scan matching limit) |

---

## Limitations & Solutions

### ⚠️ Problem 1: Featureless Environments

**Issue:** Long empty hallways have few laser features  
**Solution:**
- Drive slower
- Use IMU for rotation
- Add more scan-matching iterations
- Consider adding visual odometry (camera)

### ⚠️ Problem 2: Fast Motion

**Issue:** Robot moves too fast between scans  
**Solution:**
- Increase LiDAR rate (10+ Hz)
- Use IMU to predict motion
- Limit robot speed
- Increase `minimum_travel_distance`

### ⚠️ Problem 3: Dynamic Environments

**Issue:** Moving people/objects confuse scan matching  
**Solution:**
- Enable outlier rejection in rf2o
- Use temporal filtering
- Consider person detection/filtering

### ⚠️ Problem 4: Rotation-Only Motion

**Issue:** Pure rotation is hard for scan matching  
**Solution:**
- **IMU is critical here!**
- IMU provides accurate rotation rate
- Combine with scan matching

---

## Minimal Example (LiDAR Only, No IMU)

If you really only have LiDAR:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MinimalOdomPublisher(Node):
    def __init__(self):
        super().__init__('minimal_odom')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_identity_tf)
        
    def publish_identity_tf(self):
        """Publish identity transform - SLAM will estimate motion"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0  # Identity
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = MinimalOdomPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Then just run SLAM:**
```bash
# Publish identity TF
ros2 run my_package minimal_odom.py

# Run SLAM (will use pure scan matching)
ros2 launch slam_toolbox online_async_launch.py
```

---

## Testing Your Setup

```bash
# 1. Verify LiDAR
ros2 topic echo /scan --once

# 2. Verify IMU (if you have one)
ros2 topic echo /imu/data --once

# 3. Run LiDAR odometry
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node

# 4. Check if odometry is being published
ros2 topic echo /odom_rf2o --once

# 5. Visualize in RViz
ros2 run rviz2 rviz2
# Add: LaserScan (/scan), Odometry (/odom_rf2o), TF
```

---

## Recommended Setup for Physical Robot

**Best approach for LiDAR + IMU:**

1. **Hardware:**
   - 2D LiDAR (RPLidar A2, Hokuyo, etc.) - 10 Hz
   - 6-axis IMU (MPU6050, BNO055, etc.) - 100 Hz
   - Onboard computer (Raspberry Pi 4, Jetson Nano, etc.)

2. **Software Stack:**
   ```
   Hardware Drivers
        ↓
   /scan + /imu/data
        ↓
   rf2o_laser_odometry (LiDAR → Odometry)
        ↓
   robot_localization (Fuse IMU + Odometry)
        ↓
   slam_toolbox (SLAM)
        ↓
   /map + Localization
   ```

3. **Launch sequence:**
   ```bash
   # Terminal 1: Drivers
   ros2 launch my_robot_bringup sensors.launch.py
   
   # Terminal 2: Odometry + Fusion
   ros2 launch my_robot_bringup odometry.launch.py
   
   # Terminal 3: SLAM
   ros2 launch slam_toolbox online_async_launch.py
   
   # Terminal 4: Visualization
   ros2 run rviz2 rviz2
   ```

---

## Summary

**Without wheel encoders, you have 3 options:**

1. **Pure scan-matching** (LiDAR only)
   - Works but less accurate
   - SLAM toolbox can do this natively

2. **LiDAR odometry** (rf2o_laser_odometry)
   - Better than pure scan-matching
   - Provides explicit odometry estimate

3. **LiDAR + IMU fusion** (Recommended!)
   - Most accurate without encoders
   - Use rf2o + robot_localization + SLAM
   - Industry standard approach

**Bottom line:** Yes, you can absolutely do SLAM without encoders! The IMU helps a lot, but even with just LiDAR it's possible.

