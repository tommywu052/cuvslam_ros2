# cuvslam_ros2 — Vision-based Navigation for Wheeled-Legged Robot

Pure-vision navigation stack using NVIDIA cuVSLAM and nvblox on ROS 2 Humble.
No LiDAR required — only an Intel RealSense depth camera.

## Features

- **Visual Odometry** (`cuvslam_odom_node.py`) — RGBD or VIO (stereo IR + IMU) mode
- **3D Mapping** (`nvblox_mapper_node.py`) — Real-time occupancy grid from depth images
- **Nav2 Navigation** (`navigation_with_cuvslam.launch.py`) — Full autonomous navigation with cuVSLAM localization
- **Mapping Launch** (`nvblox_mapping.launch.py`) — Drive-around mapping without LiDAR

## Hardware

- NVIDIA Jetson Orin NX 16GB
- Intel RealSense D435i
- Wheeled-legged robot platform with `wl_base_node`

## Dependencies

This package requires the following repos to be cloned and installed:

- [cuVSLAM](https://github.com/tommywu052/cuVSLAM) — Python bindings for NVIDIA visual SLAM

  ```bash
  cd cuVSLAM/python
  pip install -e .
  ```

- [nvblox](https://github.com/tommywu052/nvblox) — GPU-accelerated 3D mapping with PyTorch integration

  ```bash
  cd nvblox/nvblox_torch
  pip install -e .
  ```

### System Requirements

```bash
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros

pip install numpy scipy torch
```

## Quick Start

### 1. Visual Odometry Test

```bash
# Prerequisites: start RealSense camera + wl_base_node first
./start_cuvslam_test.sh          # RGBD mode (default)
./start_cuvslam_test.sh --vio    # VIO mode (stereo IR + IMU, requires USB 3.x)
```

### 2. Nvblox Mapping (build occupancy grid)

```bash
bash start_nvblox_mapping.sh
# Drive around, then save:
ros2 service call /nvblox/save_map std_srvs/srv/Trigger
```

### 3. Full Navigation

```bash
ros2 launch ~/legged_robot/cuvslam_ros2/navigation_with_cuvslam.launch.py
# Use RViz "2D Goal Pose" to send navigation goals
```

## File Overview

| File | Description |
|------|-------------|
| `cuvslam_odom_node.py` | cuVSLAM visual odometry ROS 2 node (publishes `/visual_odom` + `map→odom` TF) |
| `nvblox_mapper_node.py` | nvblox 3D mapper node (publishes `/nvblox/esdf_costmap` occupancy grid) |
| `navigation_with_cuvslam.launch.py` | Full Nav2 navigation launch (cuVSLAM + Nav2 stack) |
| `nvblox_mapping.launch.py` | Mapping-only launch (cuVSLAM + nvblox) |
| `nav2_cuvslam_params.yaml` | Nav2 parameters tuned for cuVSLAM (no AMCL) |
| `start_cuvslam_test.sh` | Quick-start script for visual odometry testing |
| `start_nvblox_mapping.sh` | Quick-start script for nvblox mapping |
| `fastdds_no_shm.xml` | FastDDS config to disable shared memory (for stability) |
| `test_cuvslam_standalone.py` | Standalone cuVSLAM API test (no ROS) |

## Related Repos

- [wheeled_legged_robot](https://github.com/tommywu052/wheeled_legged_robot) — Main robot repo (ROS 2 packages, URDF, LiDAR navigation)
- [cuVSLAM](https://github.com/tommywu052/cuVSLAM) — NVIDIA visual SLAM library
- [nvblox](https://github.com/tommywu052/nvblox) — NVIDIA GPU-accelerated 3D reconstruction
