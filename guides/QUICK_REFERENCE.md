# Quick Reference: Testing SLAM with Your Robot Data

## 🎯 Quick Start (Simplest Method)

### Terminal 1: Launch SLAM
```bash
cd ~/dev_ws
source install/setup.bash
ros2 launch minimal_slam slam_playback.launch.py
```

### Terminal 2: Play Rosbag
```bash
cd ~/dev_ws/src/minimal_slam/robot_data
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3
```

### Terminal 3: Save Map (after playback)
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map
```

---

## 🔧 Advanced Setup (With IMU Fusion)

### Terminal 1: Launch SLAM + IMU Fusion
```bash
cd ~/dev_ws
source install/setup.bash
ros2 launch minimal_slam slam_playback_with_imu.launch.py
```

### Terminal 2: Play Rosbag
```bash
cd ~/dev_ws/src/minimal_slam/robot_data
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3
```

---

## 📊 Useful Commands

### Inspect Rosbag
```bash
cd ~/dev_ws/src/minimal_slam/robot_data
ros2 bag info rosbag2_2025_11_30-20_54_56_0.db3
```

### Check Topic Rates (while playing)
```bash
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic hz /imu/gyro
ros2 topic hz /imu/accel
```

### Monitor SLAM Performance
```bash
# Map updates
ros2 topic hz /map

# Robot pose
ros2 topic echo /slam_toolbox/pose

# Scan matching score
ros2 topic echo /slam_toolbox/scan_match_score
```

### Visualize TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf
```

### Check Fused Odometry (if using IMU fusion)
```bash
ros2 topic echo /odom --once
```

---

## 🎮 Playback Options

### Normal Speed
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3
```

### Slow Motion (50% speed)
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --rate 0.5
```

### Fast Forward (2x speed)
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --rate 2.0
```

### Loop Continuously
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --loop
```

### Play Only Specific Topics
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 \
  --topics /scan /odom_rf2o /tf /tf_static
```

### Start from Specific Time
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 \
  --start-offset 30  # Skip first 30 seconds
```

---

## 🛠️ Build & Install

### First Time Setup
```bash
cd ~/dev_ws
colcon build --packages-select minimal_slam
source install/setup.bash
```

### After Modifying Files
```bash
cd ~/dev_ws
colcon build --packages-select minimal_slam --symlink-install
source install/setup.bash
```

---

## 🐛 Troubleshooting Quick Fixes

### "No /imu/data topic"
**Solution:** Use `slam_playback.launch.py` (without IMU fusion) or ensure imu_combiner is running.

### "Transform timeout"
**Solution:** Make sure `use_sim_time: true` is set everywhere and bag is playing.

### "Old messages" warning
**Solution:** Add `--clock` flag:
```bash
ros2 bag play rosbag2_2025_11_30-20_54_56_0.db3 --clock
```

### Map not updating
**Check topics:**
```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
```

### RViz shows nothing
**Check Fixed Frame:** Set to `map` in RViz Global Options

---

## 📁 File Locations

### Launch Files
- `launch/slam_playback.launch.py` - Simple playback (uses existing /odom_rf2o)
- `launch/slam_playback_with_imu.launch.py` - Full fusion (combines IMU data)
- `launch/slam_with_imu.launch.py` - Live robot with simulation
- `launch/slam.launch.py` - Basic SLAM with simulation

### Config Files
- `config/slam.yaml` - SLAM Toolbox parameters
- `config/ekf.yaml` - Sensor fusion parameters

### Scripts
- `scripts/imu_combiner.py` - Combines /imu/gyro + /imu/accel

### Documentation
- `guides/ROSBAG_PLAYBACK.md` - Complete playback guide
- `guides/USAGE_IMU_FUSION.md` - IMU fusion usage
- `guides/NO_ENCODERS_GUIDE.md` - SLAM without encoders

### Robot Data
- `robot_data/rosbag2_2025_11_30-20_54_56_0.db3` - Your recorded data
- `robot_data/metadata.yaml` - Rosbag metadata

---

## ⚡ One-Liner Commands

### Quick Test
```bash
ros2 launch minimal_slam slam_playback.launch.py & sleep 3 && ros2 bag play ~/dev_ws/src/minimal_slam/robot_data/rosbag2_2025_11_30-20_54_56_0.db3
```

### Quick Map Save
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/robot_map_$(date +%Y%m%d_%H%M%S)
```

### Monitor Everything
```bash
watch -n 1 'ros2 topic hz /scan /odom /map 2>&1 | head -20'
```

---

## 📊 Your Rosbag Stats

- **Duration:** 175 seconds (2 min 55 sec)
- **Messages:** 17,110 total
- **Key Topics:**
  - `/scan`: 1,338 msgs (~7.6 Hz)
  - `/odom_rf2o`: 1,337 msgs (~7.6 Hz)
  - `/imu/gyro`: 3,336 msgs (~19 Hz)
  - `/imu/accel`: 3,337 msgs (~19 Hz)

---

## 🎯 Expected Results

After running SLAM on your rosbag, you should get:
- ✅ Complete 2D occupancy grid map
- ✅ ~175 seconds of trajectory
- ✅ Consistent localization
- ✅ Loop closures (if robot revisited areas)
- ✅ Map file ready for navigation

---

## 🚀 Next Steps

1. **Test with this data** → Verify SLAM works
2. **Tune parameters** → Optimize for your robot
3. **Run on live robot** → Deploy for real-time mapping
4. **Add navigation** → Use map for autonomous navigation

---

**Need help?** See `guides/ROSBAG_PLAYBACK.md` for detailed documentation.

