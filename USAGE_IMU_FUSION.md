# LiDAR + IMU Fusion SLAM - Usage Guide

## 🎯 What is This?

This implements **SLAM without wheel encoders** using:
- **LiDAR** for measuring distance to walls
- **IMU** for measuring rotation and orientation
- **rf2o** for estimating motion from laser scans (replaces encoders!)
- **robot_localization** for fusing LiDAR + IMU data
- **SLAM Toolbox** for building the map

## 📦 Installation

Install required packages:

```bash
sudo apt install ros-humble-rf2o-laser-odometry \
                 ros-humble-robot-localization
```

Build the package:

```bash
cd ~/dev_ws
colcon build --packages-select minimal_slam
source install/setup.bash
```

## 🚀 Quick Start

### 1. Launch Everything (Simulation)

```bash
ros2 launch minimal_slam slam_with_imu.launch.py
```

This starts:
- ✅ Gazebo world
- ✅ Robot with LiDAR + IMU
- ✅ LiDAR odometry (rf2o)
- ✅ Sensor fusion (EKF)
- ✅ SLAM Toolbox
- ✅ RViz

### 2. Control the Robot

In a **new terminal**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Drive Around

Use keyboard to explore:
- `i` = forward
- `,` = backward  
- `j` = turn left
- `l` = turn right
- `k` = stop

### 4. Visualize in RViz

Add these displays in RViz:
1. **TF** - See robot frames
2. **LaserScan** - Topic: `/scan`
3. **Map** - Topic: `/map`
4. **Odometry** - Topic: `/odom`

Set **Fixed Frame** to `map`.

### 5. Save the Map

When done mapping:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## 🔍 How It Works

### Pipeline Overview

```
┌─────────┐
│  LiDAR  │──────┐
└─────────┘      │
                 ├──→ rf2o_laser_odometry ──→ /odom_rf2o
┌─────────┐      │                                │
│   IMU   │──────┘                                │
└─────────┘                                       ↓
                                          robot_localization (EKF)
                                                  │
                                                  ↓
                                            /odom (fused)
                                                  │
                                                  ↓
                                            SLAM Toolbox
                                                  │
                                                  ↓
                                               /map
```

### Topics and Transforms

**Topics:**
- `/scan` - LiDAR laser scan data
- `/imu/data` - IMU orientation & angular velocity
- `/odom_rf2o` - Odometry estimated from laser scans
- `/odom` - Fused odometry (LiDAR + IMU)
- `/map` - SLAM-generated map

**TF Tree:**
```
map
 └─ odom (published by EKF)
     └─ base_footprint
         ├─ base_link
         │   ├─ laser
         │   └─ imu_link
         ├─ rear_left_wheel
         └─ rear_right_wheel
```

## 🛠️ Debugging

### Check Topics Are Publishing

```bash
# LiDAR
ros2 topic echo /scan --once

# IMU
ros2 topic echo /imu/data --once

# LiDAR odometry
ros2 topic echo /odom_rf2o --once

# Fused odometry
ros2 topic echo /odom --once
```

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
# Open frames.pdf to see the full TF tree
```

### Monitor Sensor Fusion

```bash
# See what the EKF is doing
ros2 topic echo /diagnostics
```

### Common Issues

**Problem:** No `/odom_rf2o` topic

**Solution:** Make sure `/scan` is publishing with sufficient features (walls, obstacles). rf2o needs geometry to match against.

---

**Problem:** "Transform timeout" errors

**Solution:** Check that all sensors are publishing:
```bash
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /odom_rf2o
```

---

**Problem:** Poor odometry estimates

**Solution:** 
1. Increase LiDAR update rate (already 10 Hz)
2. Add more features to the environment (walls, boxes)
3. Tune EKF parameters in `config/ekf.yaml`

## 🎛️ Configuration Files

### `config/ekf.yaml`
Configures sensor fusion:
- Which sensors to use (LiDAR odometry, IMU)
- Which data fields to fuse (position, velocity, orientation)
- Noise covariance (how much to trust each sensor)

### `config/slam.yaml`
Configures SLAM:
- Map resolution
- Scan matching parameters
- Loop closure settings

## 📊 Performance Tips

1. **LiDAR update rate**: 10 Hz is good. Higher = better odometry but more CPU.
2. **IMU update rate**: 100 Hz is plenty. Most IMUs can do 50-200 Hz.
3. **rf2o frequency**: Set to match LiDAR rate (10 Hz in our case).
4. **EKF frequency**: 30 Hz balances accuracy and CPU usage.

## 🔄 Adapting to Your Physical Robot

When moving to a real robot, edit `slam_with_imu.launch.py`:

### Remove Simulation Nodes:
```python
# Comment out or remove:
# - gazebo
# - spawn
# - robot_state_publisher (may need to keep for URDF)
```

### Keep These Nodes:
```python
# Keep:
# - rf2o (LiDAR odometry)
# - ekf (sensor fusion)
# - slam (SLAM Toolbox)
# - rviz (visualization)
```

### Add Your Hardware Drivers:
```python
# Add nodes to launch your:
# - LiDAR driver (publishes /scan)
# - IMU driver (publishes /imu/data)
# - Motor controller (subscribes to /cmd_vel)
```

## 🧪 Testing

### Test LiDAR Odometry Only

```bash
# Launch only rf2o
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args -p laser_scan_topic:=/scan

# In another terminal, echo odometry
ros2 topic echo /odom_rf2o
```

### Test IMU Only

```bash
ros2 topic echo /imu/data
```

### Test Sensor Fusion

```bash
# Launch everything, then check fused output
ros2 topic echo /odom
```

## 📚 More Information

- **rf2o documentation**: [github.com/MAPIRlab/rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
- **robot_localization**: [docs.ros.org/en/humble/p/robot_localization](https://docs.ros.org/en/humble/p/robot_localization/index.html)
- **SLAM Toolbox**: [github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

**🎉 You're ready to do SLAM without wheel encoders!**

