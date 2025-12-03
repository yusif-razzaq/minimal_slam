# Testing SLAM with Recorded Robot Data

## 📦 Your Rosbag Overview

**File:** `robot_data/rosbag2_2025_11_30-20_54_56_0.db3`  
**Duration:** ~175 seconds (2 minutes 55 seconds)  
**Total Messages:** 17,110

### Key Topics for SLAM:

| Topic | Type | Messages | Rate | Usage |
|-------|------|----------|------|-------|
| `/scan` | LaserScan | 1,338 | ~7.6 Hz | ✅ Primary sensor for SLAM |
| `/odom_rf2o` | Odometry | 1,337 | ~7.6 Hz | ✅ Already computed odometry! |
| `/tf` | TFMessage | 1,337 | ~7.6 Hz | ✅ Transform data |
| `/imu/gyro` | Vector3 | 3,336 | ~19 Hz | ⚠️ Format issue (not sensor_msgs/Imu) |
| `/imu/accel` | Vector3 | 3,337 | ~19 Hz | ⚠️ Format issue (not sensor_msgs/Imu) |

---

## 🚀 Quick Start: Test SLAM with Your Data

### Step 1: Build and Source

```bash
cd ~/dev_ws
colcon build --packages-select minimal_slam
source install/setup.bash
```

### Step 2: Launch SLAM (in Terminal 1)

```bash
ros2 launch minimal_slam slam_playback.launch.py
```

This starts:
- ✅ Robot state publisher (for TF frames)
- ✅ SLAM Toolbox (listening for `/scan` and `/odom_rf2o`)
- ✅ RViz (for visualization)

### Step 3: Play Your Rosbag (in Terminal 2)

```bash
cd ~/dev_ws/src/minimal_slam/robot_data

# Play at normal speed
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3

# Or play slower for better visualization (0.5x speed)
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --rate 0.5

# Or play in a loop for testing
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --loop
```

**Important:** The rosbag contains `/odom_rf2o`, so you don't need to run rf2o separately!

### Step 4: Visualize in RViz

Add these displays in RViz:

1. **TF** - See robot frames
   - Fixed Frame: `map`

2. **LaserScan**
   - Topic: `/scan`
   - Color: By intensity or range

3. **Map**
   - Topic: `/map`
   - Color Scheme: map

4. **Odometry**
   - Topic: `/odom_rf2o` (or `/odom` if using EKF)
   - Show trail

5. **Path** (optional)
   - Topic: `/slam_toolbox/graph_visualization`

### Step 5: Watch the Map Build!

You should see:
- ✅ Laser scans appearing in real-time
- ✅ Map gradually building as the robot moves
- ✅ Robot pose updating based on odometry

### Step 6: Save the Map

After playback completes:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map
```

This creates:
- `my_robot_map.pgm` - The map image
- `my_robot_map.yaml` - Map metadata

---

## 🔧 Advanced: Using IMU Data

Your bag has IMU data, but it's in separate topics as raw `Vector3` messages. To use it with the EKF, you need to combine and convert it.

### Option 1: Create an IMU Fusion Node

Create a node that subscribes to `/imu/gyro` and `/imu/accel` and publishes combined `sensor_msgs/Imu` on `/imu/data`:

```python
#!/usr/bin/env python3
"""
Combine separate gyro/accel topics into sensor_msgs/Imu
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ImuCombiner(Node):
    def __init__(self):
        super().__init__('imu_combiner')
        
        # QoS for sensor data
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.gyro_sub = self.create_subscription(
            Vector3, '/imu/gyro', self.gyro_callback, qos)
        self.accel_sub = self.create_subscription(
            Vector3, '/imu/accel', self.accel_callback, qos)
        
        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)
        
        # Storage
        self.latest_gyro = None
        self.latest_accel = None
        
        self.get_logger().info('IMU Combiner started')
    
    def gyro_callback(self, msg):
        self.latest_gyro = msg
        self.publish_combined()
    
    def accel_callback(self, msg):
        self.latest_accel = msg
        self.publish_combined()
    
    def publish_combined(self):
        if self.latest_gyro is None or self.latest_accel is None:
            return
        
        # Create combined IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Angular velocity (from gyro)
        imu_msg.angular_velocity = self.latest_gyro
        imu_msg.angular_velocity_covariance[0] = 0.02
        imu_msg.angular_velocity_covariance[4] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.02
        
        # Linear acceleration (from accel)
        imu_msg.linear_acceleration = self.latest_accel
        imu_msg.linear_acceleration_covariance[0] = 0.04
        imu_msg.linear_acceleration_covariance[4] = 0.04
        imu_msg.linear_acceleration_covariance[8] = 0.04
        
        # Orientation (unknown - set covariance to -1)
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance[0] = -1.0
        
        self.imu_pub.publish(imu_msg)


def main():
    rclpy.init()
    node = ImuCombiner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Save this as `scripts/imu_combiner.py` and make it executable.

### Option 2: Use Only LiDAR Odometry (Simpler)

Since your bag already has `/odom_rf2o`, you can skip the IMU fusion entirely! The RF2O odometry is sufficient for SLAM.

---

## 📊 Analyzing Your Rosbag

### Inspect Topics

```bash
cd ~/dev_ws/src/minimal_slam/robot_data
ros2 bag info rosbag2_2025_11_30-20_54_56_0.db3
```

### Echo Specific Topics

```bash
# Check laser scan
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --topics /scan

# In another terminal
ros2 topic echo /scan --once
```

### Check Message Rates

```bash
# Play bag
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3

# In another terminal, check rates
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic hz /imu/gyro
```

### Visualize Transforms

```bash
# Play bag
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3

# In another terminal
ros2 run tf2_tools view_frames
# This creates frames.pdf showing the TF tree
```

---

## 🎛️ Configuration for Your Robot Data

### Update SLAM Config (Optional)

If you want to tune SLAM for your specific data, edit `config/slam.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # ... existing params ...
    
    # Adjust based on your robot's actual behavior
    minimum_travel_distance: 0.15  # Meters between updates
    minimum_travel_heading: 0.15   # Radians between updates
    
    # Tune for your sensor noise
    max_laser_range: 8.0  # Adjust based on your LiDAR
    minimum_time_interval: 0.13  # ~7.6 Hz from your data
```

### Update EKF Config (If Using IMU)

If you create the IMU combiner node, edit `config/ekf.yaml` to use `/imu/data`:

```yaml
ekf_filter_node:
  ros__parameters:
    # ... existing params ...
    
    imu0: /imu/data  # Your combined IMU topic
    imu0_config: [false, false, false,
                  false, false, false,  # No orientation (not computed)
                  false, false, false,
                  false, false, true,   # Use yaw rate only
                  false, false, false]
```

---

## 🐛 Troubleshooting

### Problem: "No map updates" or "SLAM not working"

**Check:**
```bash
# Is /scan being published?
ros2 topic hz /scan

# Is /odom_rf2o being published?
ros2 topic hz /odom_rf2o

# Are timestamps correct?
ros2 topic echo /scan --once | grep stamp
```

**Solution:** Make sure you're using `use_sim_time: true` and the bag is playing.

---

### Problem: "Transform timeout" errors

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
```

**Solution:** Ensure `robot_state_publisher` is running and your URDF defines all necessary frames.

---

### Problem: "Old messages" warning

This happens when bag playback is paused or rate-limited.

**Solution:** Use `--clock` flag to publish clock:
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --clock
```

---

### Problem: Map looks distorted or jumpy

**Possible causes:**
1. Poor odometry quality
2. Insufficient laser features
3. Fast motion in the recording

**Solutions:**
- Try playing at slower speed: `--rate 0.5`
- Tune SLAM parameters
- Check if odometry is continuous (no jumps)

---

## 📈 Performance Metrics

Monitor SLAM performance:

```bash
# SLAM pose (where SLAM thinks the robot is)
ros2 topic echo /slam_toolbox/pose

# Scan matching score (higher = better)
ros2 topic echo /slam_toolbox/scan_match_score

# Number of nodes in the graph
ros2 topic echo /slam_toolbox/graph_node_count
```

---

## 🎯 Expected Results

With your ~3 minute recording, you should:

1. ✅ Build a complete 2D map of the environment
2. ✅ See consistent localization throughout
3. ✅ Have no major drift (SLAM corrects via loop closure)
4. ✅ Generate a usable map for navigation

**Good indicators:**
- Map is coherent and recognizable
- Walls are straight and aligned
- Loop closures happen (if robot revisits areas)
- Final pose is close to initial pose (if it's a loop)

---

## 🔄 Next Steps

### 1. Test with Live Robot

Once SLAM works with recorded data, test with live sensors:

```bash
# Launch on your robot
ros2 launch minimal_slam slam_with_imu.launch.py
```

Remove the Gazebo nodes from the launch file first!

### 2. Integrate with Navigation

Use your saved map for navigation:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  map:=~/my_robot_map.yaml
```

### 3. Continuous Mapping

For long-term mapping, enable serialization in `slam.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    map_file_name: /home/user/maps/my_map
    map_start_pose: [0.0, 0.0, 0.0]
    mode: mapping  # or 'localization' for existing map
```

---

## 📝 Summary

### What Your Rosbag Provides:

✅ **Essential for SLAM:**
- `/scan` - Laser data for mapping
- `/odom_rf2o` - Already computed odometry (great!)
- `/tf` - Transform relationships

⚠️ **Nice to Have (but needs conversion):**
- `/imu/gyro` + `/imu/accel` - Can be combined for IMU fusion

### Recommended Workflow:

1. **Start Simple:** Use `slam_playback.launch.py` with existing RF2O odometry
2. **Visualize:** Watch map build in RViz
3. **Save Map:** Use map_saver_cli when satisfied
4. **Optional:** Add IMU fusion if you want better orientation estimates

### Your rosbag is SLAM-ready! 🎉

The fact that it already contains `/odom_rf2o` means someone ran rf2o during recording, which is perfect. You can start testing SLAM immediately without any additional processing.

