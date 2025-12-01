# Physical Robot Integration Guide

## Overview

This SLAM system requires **3 main data streams** to work:

```
Physical Robot
    ├─ LiDAR Sensor      → publishes /scan
    ├─ Wheel Encoders    → publishes /odom
    └─ Robot Description → publishes /robot_description + TF tree
                ↓
           SLAM Toolbox
                ↓
           Map + Localization
```

---

## Required Data Streams

### 1. **Laser Scan** (`/scan` topic)

**What:** 2D laser distance measurements  
**Type:** `sensor_msgs/LaserScan`  
**Rate:** 5-10 Hz minimum (higher is better)  
**Publisher:** Your LiDAR sensor

**Message Structure:**
```yaml
header:
  stamp: current_time
  frame_id: "laser"           # Name of your LiDAR frame
angle_min: -3.14              # Start angle (radians)
angle_max: 3.14               # End angle (radians)
angle_increment: 0.0174       # Angular step (radians)
range_min: 0.3                # Minimum valid range (meters)
range_max: 10.0               # Maximum valid range (meters)
ranges: [1.2, 1.3, ...]       # Array of distance measurements
```

**How to integrate:**
```bash
# Your robot must publish this topic
ros2 topic pub /scan sensor_msgs/LaserScan "..." --rate 10
```

**Requirements:**
- Must provide 360° coverage (or at least 180°)
- Consistent timing (regular intervals)
- Valid range data (no NaNs or Infs for obstacles)

---

### 2. **Odometry** (`/odom` topic)

**What:** Robot's estimated position based on wheel encoders  
**Type:** `nav_msgs/Odometry`  
**Rate:** 20-50 Hz (higher is better for smooth motion)  
**Publisher:** Your robot's motor controller / base driver

**Message Structure:**
```yaml
header:
  stamp: current_time
  frame_id: "odom"            # Odometry reference frame
child_frame_id: "base_footprint"  # Robot base frame
pose:
  pose:
    position:
      x: 0.0                  # X position (meters)
      y: 0.0                  # Y position (meters)
      z: 0.0                  # Z position (usually 0 for 2D)
    orientation:              # Quaternion rotation
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
twist:
  twist:
    linear:
      x: 0.5                  # Forward velocity (m/s)
      y: 0.0                  # Lateral velocity (usually 0)
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.1                  # Rotation velocity (rad/s)
```

**How to integrate:**
```python
# Example: Publishing odometry from wheel encoders
import rclpy
from nav_msgs.msg import Odometry

odom_pub = node.create_publisher(Odometry, '/odom', 10)

# Calculate from encoder ticks
odom = Odometry()
odom.header.stamp = node.get_clock().now().to_msg()
odom.header.frame_id = 'odom'
odom.child_frame_id = 'base_footprint'
odom.pose.pose.position.x = x_pos
odom.pose.pose.position.y = y_pos
# ... set orientation and velocities
odom_pub.publish(odom)
```

**Requirements:**
- Must be continuous (no gaps)
- Should drift slowly (good encoder resolution)
- SLAM will correct drift using laser scans

---

### 3. **TF Transforms** (Transform Tree)

**What:** Spatial relationships between robot parts  
**Publisher:** `robot_state_publisher` + your odometry publisher  
**Rate:** Same as odometry (20-50 Hz)

**Required Transform Chain:**
```
map
 └─ odom                    ← Published by SLAM Toolbox
     └─ base_footprint      ← Published by your odometry
         └─ base_link       ← Published by robot_state_publisher
             └─ laser       ← Published by robot_state_publisher
```

**How TF works:**
1. **You publish:** `odom` → `base_footprint` (from wheel encoders)
2. **You publish:** `base_footprint` → `laser` (static, from URDF)
3. **SLAM publishes:** `map` → `odom` (drift correction)

**How to publish TF:**
```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

tf_broadcaster = TransformBroadcaster(node)

# Broadcast odom -> base_footprint
t = TransformStamped()
t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = 'odom'
t.child_frame_id = 'base_footprint'
t.transform.translation.x = x_pos
t.transform.translation.y = y_pos
t.transform.rotation = quaternion_from_euler(0, 0, theta)
tf_broadcaster.sendTransform(t)
```

---

## Robot Description (URDF)

**What:** Description of your robot's physical structure  
**Where:** URDF/xacro file describing links and joints  
**Used for:** Static transforms (like LiDAR position on robot)

**Minimal URDF for physical robot:**
```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_footprint"/>
  
  <link name="base_link">
    <!-- Your robot body -->
  </link>
  
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1"/>  <!-- Height off ground -->
  </joint>
  
  <link name="laser">
    <!-- Your LiDAR sensor -->
  </link>
  
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0.2 0 0.3"/>  <!-- LiDAR position: X, Y, Z -->
  </joint>
</robot>
```

---

## SLAM Configuration

The SLAM parameters are in `config/slam.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Frame names (must match your robot!)
    odom_frame: odom              # Your odometry frame
    map_frame: map                # SLAM's global frame
    base_frame: base_footprint    # Your robot's base
    scan_topic: /scan             # Your LiDAR topic
    
    # Performance
    resolution: 0.05              # Map cell size (5cm)
    max_laser_range: 10.0         # Use scans up to 10m
    
    # Tuning
    minimum_travel_distance: 0.2  # Update every 20cm
    minimum_travel_heading: 0.2   # Or every 0.2 radians
```

---

## Integration Checklist

### Step 1: Hardware Setup
- [ ] LiDAR sensor installed and working
- [ ] Wheel encoders providing position data
- [ ] Measure LiDAR position relative to robot center

### Step 2: Create URDF
- [ ] Create URDF file with your robot dimensions
- [ ] Add `laser` link at correct position
- [ ] Test: `ros2 launch robot_state_publisher ...`

### Step 3: Publish Odometry
- [ ] Create node to read wheel encoders
- [ ] Calculate robot position (x, y, theta)
- [ ] Publish `/odom` topic at 20-50 Hz
- [ ] Publish TF: `odom` → `base_footprint`

### Step 4: Publish Laser Scans
- [ ] Configure LiDAR to publish `/scan`
- [ ] Verify data: `ros2 topic echo /scan`
- [ ] Check frame_id matches your URDF

### Step 5: Run SLAM
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=config/slam.yaml \
  use_sim_time:=false
```

### Step 6: Verify
```bash
# Check topics
ros2 topic list

# Should see:
# /scan       ← Your LiDAR
# /odom       ← Your encoders
# /map        ← SLAM output
# /tf         ← Transforms

# Check TF tree
ros2 run tf2_tools view_frames
# Should show: map → odom → base_footprint → laser
```

---

## Example: Minimal Physical Robot Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class MinimalRobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot state (from encoders)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Timers
        self.create_timer(0.02, self.publish_odometry)  # 50 Hz
        self.create_timer(0.1, self.publish_scan)        # 10 Hz
        
    def publish_odometry(self):
        """Read encoders and publish odometry"""
        # TODO: Read from your wheel encoders
        # self.x, self.y, self.theta = calculate_from_encoders()
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # Set orientation from theta...
        self.odom_pub.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        # Set rotation from theta...
        self.tf_broadcaster.sendTransform(t)
        
    def publish_scan(self):
        """Read LiDAR and publish scan"""
        # TODO: Read from your LiDAR
        # ranges = read_lidar()
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -3.14
        scan.angle_max = 3.14
        scan.angle_increment = 0.0174
        scan.range_min = 0.3
        scan.range_max = 10.0
        scan.ranges = ranges  # Your LiDAR data
        self.scan_pub.publish(scan)

def main():
    rclpy.init()
    node = MinimalRobotDriver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## What SLAM Does

**Input:**
- `/scan` - Laser measurements
- `/odom` - Encoder-based position (drifts over time)
- TF tree - Sensor positions

**Processing:**
- Matches laser scans to previous scans
- Detects loop closures (revisiting same place)
- Corrects odometry drift

**Output:**
- `/map` - 2D occupancy grid (walls, obstacles)
- TF: `map` → `odom` - Drift correction
- Accurate robot position in global map

---

## Key Differences: Simulation vs Physical Robot

| Aspect | Simulation | Physical Robot |
|--------|-----------|----------------|
| Odometry | Perfect (from Gazebo) | Drifts (from encoders) |
| LiDAR | No noise | Has noise/uncertainty |
| TF | Auto-published | You must publish |
| use_sim_time | true | **false** |
| Clock | Gazebo time | System time |

**Important:** Set `use_sim_time:=false` for physical robots!

---

## Common Physical Robot Platforms

### For ROS2 Robots:
- **TurtleBot 4** - Has all sensors built-in
- **ROSbot 2.0** - LiDAR + encoders ready
- **Custom robot** - Need to implement above interfaces

### Sensor Recommendations:
- **LiDAR:** RPLidar A1/A2, Hokuyo, SICK
- **Encoders:** Quadrature encoders on wheels
- **IMU:** (Optional) Helps with odometry

---

## Testing Your Integration

```bash
# 1. Check if topics exist
ros2 topic list

# 2. Verify scan data
ros2 topic echo /scan --once

# 3. Verify odometry
ros2 topic echo /odom --once

# 4. Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# 5. Visualize in RViz
ros2 run rviz2 rviz2
# Add: RobotModel, LaserScan (/scan), Map (/map), TF
# Set Fixed Frame: map
```

---

## Summary

**You need to provide:**
1. `/scan` topic (LiDAR data) - 5-10 Hz
2. `/odom` topic (wheel encoders) - 20-50 Hz  
3. TF: `odom` → `base_footprint` - 20-50 Hz
4. URDF describing robot geometry
5. Set `use_sim_time: false`

**SLAM provides:**
1. `/map` topic (occupancy grid)
2. TF: `map` → `odom` (drift correction)
3. Accurate localization

**That's it!** The SLAM algorithm handles everything else automatically.

