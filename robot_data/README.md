# Robot Data - Recorded Sensor Data

This directory contains real sensor data recorded from your robot for testing SLAM.

## 📦 Contents

### Rosbag File
**File:** `rosbag2_2025_11_30-20_54_56_0.db3` (11 MB)  
**Format:** SQLite3 (ROS 2 rosbag format)  
**Duration:** 175.66 seconds (~2 minutes 55 seconds)  
**Total Messages:** 17,110

### Metadata File
**File:** `metadata.yaml`  
Contains rosbag information including topics, message counts, and QoS profiles.

---

## 📊 Topic Breakdown

| Topic | Type | Count | Rate | Purpose |
|-------|------|-------|------|---------|
| `/scan` | LaserScan | 1,338 | 7.6 Hz | **Primary SLAM sensor** |
| `/odom_rf2o` | Odometry | 1,337 | 7.6 Hz | **Precomputed odometry** |
| `/tf` | TFMessage | 1,337 | 7.6 Hz | Transform tree |
| `/tf_static` | TFMessage | 2 | - | Static transforms |
| `/imu/gyro` | Vector3 | 3,336 | 19 Hz | Gyroscope data |
| `/imu/accel` | Vector3 | 3,337 | 19 Hz | Accelerometer data |
| `/rosout` | Log | 6,242 | 35.5 Hz | System logs |
| `/rover/armed` | Bool | 175 | 1 Hz | Robot state |
| `/parameter_events` | ParameterEvent | 0 | - | ROS 2 parameters |
| `/connected_clients` | ConnectedClients | 2 | - | Rosbridge clients |
| `/client_count` | Int32 | 4 | - | Client count |
| `/events/write_split` | WriteSplitEvent | 0 | - | Rosbag events |

---

## ✅ SLAM-Ready Data

Your rosbag is **ready for SLAM testing** with the following essential data:

### 1. **LiDAR Data** (`/scan`)
- ✅ 1,338 scans at ~7.6 Hz
- ✅ Sufficient for building a map
- ✅ Good update rate for scan matching

### 2. **Odometry Data** (`/odom_rf2o`)
- ✅ Already computed from laser scans
- ✅ Matches scan frequency
- ✅ Ready to use directly

### 3. **Transform Data** (`/tf`, `/tf_static`)
- ✅ Contains coordinate frame relationships
- ✅ Needed for proper sensor alignment

### 4. **IMU Data** (Optional Enhancement)
- ⚠️ Split into separate topics (`/imu/gyro`, `/imu/accel`)
- ⚠️ Needs combination into `sensor_msgs/Imu` format
- ✅ Can be used with `imu_combiner.py` script

---

## 🚀 How to Use This Data

### Method 1: Simple SLAM (Recommended to Start)

Uses existing `/odom_rf2o` directly:

```bash
# Terminal 1: Launch SLAM
ros2 launch minimal_slam slam_playback.launch.py

# Terminal 2: Play rosbag
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3
```

### Method 2: With IMU Fusion (Advanced)

Combines IMU data for better accuracy:

```bash
# Terminal 1: Launch SLAM with IMU fusion
ros2 launch minimal_slam slam_playback_with_imu.launch.py

# Terminal 2: Play rosbag
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3
```

---

## 🔍 Inspecting the Data

### View Rosbag Info
```bash
cd ~/dev_ws/src/minimal_slam/robot_data
ros2 bag info rosbag2_2025_11_30-20_54_56_0.db3
```

### Play and Echo Topics
```bash
# Terminal 1: Play bag
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3

# Terminal 2: Check specific topic
ros2 topic echo /scan --once
ros2 topic echo /odom_rf2o --once
ros2 topic echo /imu/gyro --once
```

### Check Message Rates
```bash
# Terminal 1: Play bag
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3

# Terminal 2: Monitor rates
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic hz /imu/gyro
```

### Visualize in RViz
```bash
# Terminal 1: Start RViz
ros2 run rviz2 rviz2

# Terminal 2: Play bag
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3

# In RViz, add:
# - LaserScan (topic: /scan)
# - Odometry (topic: /odom_rf2o)
# - TF
```

---

## 📝 Recording Details

### QoS Profiles

- **Most topics:** Reliable, Transient Local (history depth 0)
- **IMU topics:** Best Effort, Transient Local
- **Rosout:** Reliable, Volatile (10 second lifespan)

### Recording Command (Reference)

This data was likely recorded with a command similar to:

```bash
ros2 bag record \
  /scan /odom_rf2o /tf /tf_static \
  /imu/gyro /imu/accel \
  /rover/armed \
  -o rosbag2_2025_11_30-20_54_56
```

---

## 🎯 What You Can Test

With this rosbag, you can:

1. ✅ **Test SLAM pipeline** - Build a map from recorded data
2. ✅ **Validate odometry** - Check if RF2O worked correctly
3. ✅ **Tune SLAM parameters** - Optimize without re-recording
4. ✅ **Debug issues** - Replay data to find problems
5. ✅ **Compare approaches** - Test with/without IMU fusion
6. ✅ **Develop algorithms** - Work offline without robot

---

## 💡 Tips for Best Results

### Playback Speed
- **Normal (1.0x):** Realistic real-time performance
- **Slow (0.5x):** Better visualization, easier debugging
- **Fast (2.0x):** Quick iteration, but may stress SLAM

```bash
# Slow motion
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --rate 0.5

# Fast forward
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --rate 2.0
```

### Use Sim Time
Always set `use_sim_time: true` when playing bags:
- Prevents "old message" warnings
- Ensures proper timestamp synchronization
- Required for SLAM Toolbox temporal consistency

### Loop for Testing
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --loop
```

---

## 🐛 Known Issues

### 1. IMU Format Mismatch
**Issue:** IMU data is split into `/imu/gyro` and `/imu/accel` as `Vector3` messages.  
**Solution:** Use `imu_combiner.py` script or `slam_playback_with_imu.launch.py`.

### 2. No Odometry (if you delete /odom_rf2o)
**Issue:** SLAM needs odometry source.  
**Solution:** Either keep `/odom_rf2o` or run `rf2o_laser_odometry` on `/scan`.

---

## 📊 Expected Map Quality

Based on the data characteristics:
- **Scan rate:** 7.6 Hz → ✅ Good for SLAM
- **Duration:** ~3 minutes → ✅ Sufficient for room/area map
- **Odometry available:** Yes → ✅ Better accuracy
- **Loop closure potential:** Depends on robot path

---

## 🔄 Recording New Data

To record similar data from your robot:

```bash
# Record essential topics for SLAM
ros2 bag record \
  /scan \
  /odom_rf2o \
  /tf \
  /tf_static \
  /imu/gyro \
  /imu/accel \
  -o my_robot_data_$(date +%Y_%m_%d-%H_%M_%S)
```

Or record everything (larger file):

```bash
ros2 bag record -a -o my_robot_data_$(date +%Y_%m_%d-%H_%M_%S)
```

---

## 📚 Documentation

For more information, see:
- `../guides/ROSBAG_PLAYBACK.md` - Complete playback guide
- `../guides/QUICK_REFERENCE.md` - Command cheat sheet
- `../guides/USAGE_IMU_FUSION.md` - IMU integration guide

---

**Ready to test SLAM with your real robot data!** 🚀

