#!/bin/bash
# Start robot_state_publisher + cuVSLAM odom node for testing.
# Prerequisites: RealSense and wl_base_node already running.
#
# Usage:
#   ./start_cuvslam_test.sh          # RGBD mode (default)
#   ./start_cuvslam_test.sh --vio    # VIO mode (stereo IR + IMU, requires USB 3.x)
#
# Camera mount offset (base_link → camera_link):
CAMERA_X=0.135  # LiDAR(4.5cm) + 9cm forward = 13.5cm from base_link
CAMERA_Y=0.00   # centered left-right
CAMERA_Z=0.054  # LiDAR(10.4cm) - 5cm below = 5.4cm above base_link
CAMERA_YAW=0.0
CAMERA_PITCH=0.0
CAMERA_ROLL=0.0

# Parse arguments
USE_IMU="false"
if [[ "$1" == "--vio" ]]; then
    USE_IMU="true"
    echo "[INFO] VIO mode enabled (stereo IR + IMU)"
fi

source /opt/ros/humble/setup.bash
source ~/legged_robot/ROS2_Packages/install/setup.bash 2>/dev/null

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[INFO] Camera mount: x=${CAMERA_X} y=${CAMERA_Y} z=${CAMERA_Z}"
echo "[INFO] Mode: $([ $USE_IMU = 'true' ] && echo 'VIO' || echo 'RGBD')"
echo "[INFO] Starting all nodes via launch..."
python3.10 -c "
import sys, os
sys.path.insert(0, '$SCRIPT_DIR')
os.chdir('$SCRIPT_DIR')
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

urdf_path = os.path.expanduser(
    '~/legged_robot/ROS2_Packages/src/wheel_legged_urdf_pkg/urdf/wheel_legged_urdf_pkg.urdf')
with open(urdf_path) as f:
    robot_desc = f.read()

ld = LaunchDescription([
    Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen'),
    Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ['/robot_joint_states'], 'use_sim_time': False}],
        output='screen'),
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '$CAMERA_X', '--y', '$CAMERA_Y', '--z', '$CAMERA_Z',
            '--yaw', '$CAMERA_YAW', '--pitch', '$CAMERA_PITCH', '--roll', '$CAMERA_ROLL',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        output='screen'),
    ExecuteProcess(
        cmd=['python3.10', '$SCRIPT_DIR/cuvslam_odom_node.py',
             '--ros-args', '-p', 'use_imu:=$USE_IMU'],
        output='screen'),
])

ls = LaunchService()
ls.include_launch_description(ld)
ls.run()
"
