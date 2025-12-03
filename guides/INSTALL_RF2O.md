# Installing rf2o_laser_odometry for ROS2 Humble

The `rf2o_laser_odometry` package is not available as a binary for ROS2 Humble, so we need to build it from source.

## Step 1: Install robot_localization (Available as Binary)

```bash
sudo apt install ros-humble-robot-localization
```

## Step 2: Clone rf2o_laser_odometry Source

```bash
cd ~/dev_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git -b humble
```

## Step 3: Install Dependencies

```bash
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Step 4: Build Everything

```bash
cd ~/dev_ws
colcon build --packages-select rf2o_laser_odometry
colcon build --packages-select minimal_slam
source install/setup.bash
```

## Step 5: Test Installation

```bash
ros2 pkg list | grep rf2o
```

You should see: `rf2o_laser_odometry`

## Alternative: Use Simplified SLAM (No rf2o)

If you want to skip rf2o and use the built-in SLAM Toolbox scan matching instead, you can use the original `slam.launch.py` which doesn't require rf2o:

```bash
ros2 launch minimal_slam slam.launch.py
```

This will work but won't have the IMU fusion benefits.

## Quick Install Script

Run these commands in order:

```bash
# 1. Install robot_localization
sudo apt install ros-humble-robot-localization -y

# 2. Navigate to workspace
cd ~/dev_ws/src

# 3. Clone rf2o
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git -b humble

# 4. Install dependencies
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --packages-select rf2o_laser_odometry minimal_slam

# 6. Source
source install/setup.bash

# 7. Verify
ros2 pkg list | grep rf2o
```

## Troubleshooting

**Problem:** `branch 'humble' not found`

**Solution:** Try without the branch flag:
```bash
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd rf2o_laser_odometry
git checkout ros2
```

**Problem:** Build errors with rf2o

**Solution:** Make sure you have all ROS2 Humble development packages:
```bash
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-tf2-ros
```

