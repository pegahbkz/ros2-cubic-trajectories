# ROS2 Package for Cubic Trajectory Processing


## Overview
This is a ROS2-based package designed to generate, compute, and visualize cubic trajectories. 

## Installation

### 1. move all files into Your ROS2 Workspace
Navigate to your ROS2 workspace and copy all files into this directory.
Make sure you have ROS setup in the environment.
```bash
source /opt/ros/humble/setup.bash
```

### 2. Build the Package
Go back to the workspace root and build:
```bash
cd ~/ros2_ws
colcon build --packages-select ar_interface ar_test
```

### 3. Source the Environment
After building, source the setup file:
```bash
source install/setup.bash
```
*(You need to do this in every new terminal before running the package.)*

---

## Usage Instructions

### 2. Run the launch file
```bash
ros2 launch ar_test cubic_traj_gen.launch.py
```

### Run `rqt` to visualize the data
In a new terminal, start `rqt`:
```bash
rqt --force-discover
```
- Go to **Plugins → Visualization → Plot**  
- Add the following topics manually:
  ```
  /cubic_trajectory/position
  /cubic_trajectory/velocity
  /cubic_trajectory/acceleration
  ```
- The plots should update in real-time.

---

## Notes
- The package is built for **ROS2 Humble**.
- The trajectory plots are also **saved as PNG files** in `src/ar_test/ar_test/trajectory_plots/`.

---
**End of README**
