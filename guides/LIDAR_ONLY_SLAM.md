# LiDAR-Only SLAM Guide

## 🎯 Overview

This guide shows you how to run SLAM using **ONLY LiDAR data** (`/scan` topic) from your rosbag. No IMU, no wheel encoders needed!

---

## 📊 Your Rosbag Data

Based on `metadata.yaml`:

| Topic | Type | Messages | Rate | Usage |
|-------|------|----------|------|-------|
| `/scan` | LaserScan | 1,396 | ~7.7 Hz | ✅ **Used for SLAM** |
| `/cmd_vel` | Twist | 1,693 | ~9.3 Hz | ℹ️ Robot commands (not needed) |
| `/imu/gyro` | Vector3 | 3,552 | ~19.5 Hz | ❌ Not trusted/used |
| `/imu/accel` | Vector3 | 3,552 | ~19.5 Hz | ❌ Not trusted/used |

**Duration:** ~182 seconds (3 minutes)  
**Total Messages:** 15,257

### ⚠️ Important Note

Your rosbag does **NOT** contain `/odom_rf2o` like the previous one. This means we need to generate odometry from the laser scans in real-time using RF2O.

---

## 🚀 Quick Start (3 Steps)

### Step 1: Build Package

```bash
cd ~/dev_ws
colcon build --packages-select minimal_slam
source install/setup.bash
```

### Step 2: Launch SLAM (Terminal 1)

```bash
ros2 launch minimal_slam slam_lidar_only.launch.py
```

This starts:
- ✅ Robot state publisher (TF frames)
- ✅ RF2O laser odometry (generates `/odom` from `/scan`)
- ✅ SLAM Toolbox (builds map)
- ✅ RViz (visualization)

### Step 3: Play Rosbag (Terminal 2)

```bash
cd ~/dev_ws/src/minimal_slam/robot_data
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3
```

**Note:** The filename changed from the old rosbag!

---

## 📈 What to Expect

### During Playback

You should see:
1. **RF2O** processing laser scans and generating odometry
2. **SLAM Toolbox** building the map incrementally
3. **RViz** showing:
   - Laser scans (red/rainbow points)
   - Occupancy grid map (gray/white/black)
   - Robot pose and trajectory

### RF2O Console Output

```
[rf2o_laser_odometry]: Processing scan...
[rf2o_laser_odometry]: Odom: x=0.15, y=0.02, theta=0.01
```

### SLAM Console Output

```
[slam_toolbox]: Node added to graph
[slam_toolbox]: Loop closure detected!
[slam_toolbox]: Map updated
```

---

## 🎛️ RViz Setup

When RViz opens, add these displays:

### 1. TF (Transform frames)
- **Purpose:** See robot coordinate frames
- **Fixed Frame:** Set to `map`

### 2. LaserScan
- **Topic:** `/scan`
- **Color:** By Range or Intensity
- **Size:** 0.05
- **Decay Time:** 0 (for playback)

### 3. Map
- **Topic:** `/map`
- **Color Scheme:** map
- **Alpha:** 0.7

### 4. Odometry
- **Topic:** `/odom`
- **Keep:** 100 (show trail)
- **Shaft Length:** 0.5
- **Shaft Radius:** 0.05

### 5. Path (Optional)
- **Topic:** `/slam_toolbox/graph_visualization`
- Shows SLAM graph nodes

---

## 🔍 How It Works

### Pipeline

```
/scan (LaserScan)
    ↓
RF2O Laser Odometry
    ├─→ /odom (Odometry) ────→ SLAM Toolbox → /map
    └─→ TF: odom → base_footprint
```

### Key Components

1. **RF2O (Range Flow 2D Odometry)**
   - Compares consecutive laser scans
   - Estimates robot motion between scans
   - Publishes odometry on `/odom` topic
   - Updates `odom → base_footprint` transform

2. **SLAM Toolbox**
   - Uses laser scans + odometry
   - Builds occupancy grid map
   - Performs loop closure detection
   - Corrects drift over time

3. **Robot State Publisher**
   - Publishes static robot transforms
   - Defines sensor positions (laser, etc.)
   - Based on URDF model

---

## ⚙️ Configuration

### RF2O Parameters

In `slam_lidar_only.launch.py`:

```python
parameters=[{
    'laser_scan_topic': '/scan',
    'odom_topic': '/odom',
    'publish_tf': True,
    'base_frame_id': 'base_footprint',
    'odom_frame_id': 'odom',
    'freq': 10.0,
    'verbose': True,
    'use_sim_time': True
}]
```

**Key settings:**
- `publish_tf: True` - RF2O publishes the odom→base transform
- `freq: 10.0` - Update at 10 Hz (matches your scan rate)
- `use_sim_time: True` - Use rosbag timestamps

### SLAM Parameters

In `config/slam.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    scan_topic: /scan
    odom_frame: odom
    base_frame: base_footprint
    
    # Scan matching (important for LiDAR-only!)
    use_scan_matching: true
    use_scan_barycenter: true
    
    # Map resolution
    resolution: 0.05  # 5cm per pixel
    
    # Update thresholds
    minimum_travel_distance: 0.2  # meters
    minimum_travel_heading: 0.2   # radians
```

---

## 🎮 Playback Options

### Normal Speed (Real-time)
```bash
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3
```

### Slow Motion (Better for Debugging)
```bash
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3 --rate 0.5
```

### Fast Forward (Quick Testing)
```bash
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3 --rate 1.5
```

### Loop Continuously
```bash
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3 --loop
```

### With Clock Publishing
```bash
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3 --clock
```

---

## 💾 Save Your Map

After the rosbag finishes playing:

### Save Map Files
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_lidar_map
```

This creates:
- `my_lidar_map.pgm` - Map image
- `my_lidar_map.yaml` - Map metadata

### Save SLAM State (Optional)
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/yusif/maps/my_slam_session'}"
```

---

## 📊 Monitor Performance

### Check Topics

```bash
# Verify all topics are publishing
ros2 topic list

# Check rates
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
```

Expected output:
```
/scan: ~7.7 Hz
/odom: ~7.7 Hz (generated by RF2O)
/map: ~0.5 Hz (updated periodically by SLAM)
```

### Check Transforms

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_footprint
```

### Monitor RF2O

```bash
# See odometry values
ros2 topic echo /odom --once
```

### Monitor SLAM

```bash
# See robot pose estimate
ros2 topic echo /slam_toolbox/pose

# Check scan match score (higher = better)
ros2 topic echo /slam_toolbox/scan_match_score
```

---

## 🐛 Troubleshooting

### Problem: RF2O not publishing odometry

**Symptoms:**
- No `/odom` topic
- Warning: "Waiting for laser scan..."

**Check:**
```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

**Solution:**
- Ensure rosbag is playing
- Verify `/scan` topic exists and has data
- Check RF2O node is running: `ros2 node list`

---

### Problem: "Transform timeout" errors

**Symptoms:**
```
Could not transform laser scan into base_footprint
Lookup would require extrapolation into the future
```

**Solution:**
1. Make sure ALL nodes have `use_sim_time: True`
2. Check TF tree: `ros2 run tf2_tools view_frames`
3. Verify robot_state_publisher is running

---

### Problem: Map not building

**Symptoms:**
- RViz shows laser scans but no map
- SLAM Toolbox not outputting messages

**Check:**
```bash
# Is SLAM receiving scans?
ros2 topic hz /scan

# Is odometry available?
ros2 topic hz /odom

# Is map being published?
ros2 topic hz /map
```

**Solution:**
- Wait longer (map updates every 2 seconds by default)
- Check SLAM Toolbox console for errors
- Verify scan has sufficient features (not empty room)

---

### Problem: Poor map quality (wavy walls, drift)

**Possible Causes:**
1. Fast motion during recording
2. Featureless environment
3. Low scan rate
4. RF2O struggling to match scans

**Solutions:**

#### 1. Play Slower
```bash
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3 --rate 0.5
```

#### 2. Tune RF2O (Advanced)
Create a config file with tighter matching:
```yaml
# config/rf2o.yaml
/**:
  ros__parameters:
    laser_scan_topic: /scan
    odom_topic: /odom
    # Tighter convergence
    max_iterations: 10
    # More conservative motion estimates
    publish_tf: true
```

#### 3. Tune SLAM (Advanced)
Edit `config/slam.yaml`:
```yaml
slam_toolbox:
  ros__parameters:
    # Stronger scan matching
    link_match_minimum_response_fine: 0.3
    minimum_travel_distance: 0.15
    
    # More aggressive loop closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 5
```

---

### Problem: "Old messages" warning

**Symptoms:**
```
Message filter dropping message: frame 'laser' at time X
```

**Solution:**
Use `--clock` flag:
```bash
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3 --clock
```

---

## 📈 Expected Results

### Good Map Indicators
- ✅ Straight walls (not wavy)
- ✅ Closed loops (if robot revisited areas)
- ✅ Consistent features
- ✅ Minimal drift (<1m over 3 minutes)
- ✅ Clear obstacle boundaries

### Poor Map Indicators
- ❌ Doubled/ghosted walls
- ❌ Broken or disconnected features
- ❌ Large drift (robot ends far from start if it's a loop)
- ❌ Fuzzy or unclear boundaries

---

## 🎯 Performance Tips

### For Best Results

1. **Environment:**
   - Works best in structured environments (rooms, hallways)
   - Needs visible features for RF2O (walls, furniture)
   - Struggles in open spaces or long corridors

2. **Motion:**
   - Slower motion = better scan matching
   - Avoid sudden rotations
   - Smooth trajectories work best

3. **Scan Quality:**
   - Need good scan coverage (270-360°)
   - Sufficient range (~10m typical)
   - Regular update rate (5-10 Hz)

### Your Data Characteristics

| Metric | Value | Assessment |
|--------|-------|------------|
| Scan rate | 7.7 Hz | ✅ Good |
| Duration | 3 minutes | ✅ Good |
| Messages | 1,396 scans | ✅ Sufficient |
| Odometry | Generated by RF2O | ⚠️ Less accurate than encoders |

---

## 🔄 Complete Workflow

### Full Command Sequence

```bash
# 1. Build
cd ~/dev_ws
colcon build --packages-select minimal_slam
source install/setup.bash

# 2. Launch SLAM (Terminal 1)
ros2 launch minimal_slam slam_lidar_only.launch.py

# 3. Play rosbag (Terminal 2)
cd ~/dev_ws/src/minimal_slam/robot_data
ros2 bag play rosbag2_2025_12_03-00_42_27_0.db3 --rate 0.8

# 4. Save map (Terminal 3, after playback)
ros2 run nav2_map_server map_saver_cli -f ~/maps/lidar_only_map
```

---

## 📚 Understanding LiDAR-Only SLAM

### Why This Works

**RF2O Algorithm:**
1. Takes current laser scan
2. Compares to previous scan
3. Finds transformation (x, y, θ) that best aligns them
4. Publishes this as odometry

**Advantages:**
- ✅ No wheel encoders needed
- ✅ No IMU needed
- ✅ Self-contained solution
- ✅ Works on any robot with 2D LiDAR

**Limitations:**
- ⚠️ Less accurate than wheel encoders
- ⚠️ Needs textured environment (features)
- ⚠️ Can fail in featureless spaces
- ⚠️ Slower update rate

### When to Use

**Good for:**
- Indoor environments
- Rooms with furniture/walls
- Slow to moderate speeds
- Ground robots

**Not ideal for:**
- Empty hallways
- Outdoor open spaces
- High-speed motion
- 3D environments

---

## 🎓 Next Steps

### 1. Test with Current Data
Run through the quick start guide above.

### 2. Analyze Results
- Check map quality
- Look for drift
- Verify loop closures

### 3. Tune if Needed
- Adjust SLAM parameters
- Modify RF2O settings
- Try different playback speeds

### 4. Deploy on Robot (Future)
Once satisfied with rosbag results, adapt for live robot:
```bash
# Remove static_tf_pub from launch file
# Add your LiDAR driver node
# Set use_sim_time: False
```

---

## ✅ Summary

**What You Have:**
- ✅ `/scan` topic with 1,396 laser scans
- ✅ ~3 minutes of robot movement
- ✅ Sufficient data for mapping

**What We Do:**
- ✅ Generate odometry from laser scans (RF2O)
- ✅ Build map using SLAM Toolbox
- ✅ Visualize in RViz
- ✅ Save map for future use

**What You Get:**
- ✅ 2D occupancy grid map
- ✅ Robot trajectory
- ✅ Map files ready for navigation

---

**You're all set for LiDAR-only SLAM!** 🚀

Run the commands and watch your map build in RViz!

