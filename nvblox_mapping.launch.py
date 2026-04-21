#!/usr/bin/env python3
"""
Nvblox mapping launch — build an occupancy grid using cuVSLAM + nvblox.

This launch file starts cuVSLAM for pose estimation and nvblox for 3D mapping,
producing a Nav2-compatible occupancy grid entirely from the RealSense camera.
No LiDAR required.

Requires (started externally):
  - RealSense camera node
  - wl_base_node (wheel odometry for odom → base_link TF)

Usage:
  # Start mapping (drive around to build map)
  ros2 launch ~/legged_robot/cuvslam_ros2/nvblox_mapping.launch.py

  # Save the occupancy grid when done
  ros2 service call /nvblox/save_map std_srvs/srv/Trigger

  # The saved map will be at ~/maps/nvblox_map.yaml (+ .pgm + .nvblox)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    cuvslam_dir = os.path.expanduser('~/legged_robot/cuvslam_ros2')
    urdf_file = os.path.expanduser(
        '~/legged_robot/ROS2_Packages/src/wheel_legged_urdf_pkg/urdf/'
        'wheel_legged_urdf_pkg.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # --- Launch arguments ---
    declare_cam_x = DeclareLaunchArgument('camera_x', default_value='0.135')
    declare_cam_y = DeclareLaunchArgument('camera_y', default_value='0.00')
    declare_cam_z = DeclareLaunchArgument('camera_z', default_value='0.054')

    declare_voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.05',
        description='Voxel size in meters (0.05 = 5cm resolution)')
    declare_max_dist = DeclareLaunchArgument(
        'max_integration_distance', default_value='5.0',
        description='Max depth integration distance in meters')
    declare_slice_height = DeclareLaunchArgument(
        'slice_height', default_value='0.1',
        description='Height (meters) at which to slice TSDF for 2D grid')
    declare_grid_rate = DeclareLaunchArgument(
        'grid_publish_rate', default_value='1.0',
        description='Occupancy grid publish rate (Hz)')
    declare_save_name = DeclareLaunchArgument(
        'save_name', default_value='nvblox_map',
        description='Base filename for saved map')
    declare_use_imu = DeclareLaunchArgument(
        'use_imu', default_value='false',
        description='Enable VIO mode (stereo IR + IMU)')
    declare_esdf_rate = DeclareLaunchArgument(
        'esdf_publish_rate', default_value='1.0',
        description='ESDF publish rate (Hz)')
    declare_esdf_min_h = DeclareLaunchArgument(
        'esdf_slice_min_height', default_value='-0.1',
        description='ESDF bottom sampling height (m)')
    declare_esdf_max_h = DeclareLaunchArgument(
        'esdf_slice_max_height', default_value='1.5',
        description='ESDF top sampling height (m, robot top)')
    declare_mesh_rate = DeclareLaunchArgument(
        'mesh_publish_rate', default_value='0.5',
        description='Color mesh publish rate (Hz)')

    # --- Robot description ---
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['/robot_joint_states'],
            'use_sim_time': False
        }])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }])

    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'],
        output='screen')

    # --- cuVSLAM visual odometry (SLAM mode for mapping) ---
    cuvslam_node = ExecuteProcess(
        cmd=['python3.10', os.path.join(cuvslam_dir, 'cuvslam_odom_node.py'),
             '--ros-args',
             '-p', 'enable_slam:=true',
             '-p', 'publish_tf:=true',
             '-p', ['use_imu:=', LaunchConfiguration('use_imu')],
             '-p', 'auto_localize:=false'],
        output='screen')

    # --- Nvblox mapper ---
    nvblox_mapper_node = ExecuteProcess(
        cmd=['python3.10', os.path.join(cuvslam_dir, 'nvblox_mapper_node.py'),
             '--ros-args',
             '-p', ['voxel_size:=', LaunchConfiguration('voxel_size')],
             '-p', ['max_integration_distance:=', LaunchConfiguration('max_integration_distance')],
             '-p', ['slice_height:=', LaunchConfiguration('slice_height')],
             '-p', ['grid_publish_rate:=', LaunchConfiguration('grid_publish_rate')],
             '-p', ['save_name:=', LaunchConfiguration('save_name')],
             '-p', ['esdf_publish_rate:=', LaunchConfiguration('esdf_publish_rate')],
             '-p', ['esdf_slice_min_height:=', LaunchConfiguration('esdf_slice_min_height')],
             '-p', ['esdf_slice_max_height:=', LaunchConfiguration('esdf_slice_max_height')],
             '-p', ['mesh_publish_rate:=', LaunchConfiguration('mesh_publish_rate')]],
        output='screen')

    return LaunchDescription([
        # Arguments
        declare_cam_x,
        declare_cam_y,
        declare_cam_z,
        declare_voxel_size,
        declare_max_dist,
        declare_slice_height,
        declare_grid_rate,
        declare_save_name,
        declare_use_imu,
        declare_esdf_rate,
        declare_esdf_min_h,
        declare_esdf_max_h,
        declare_mesh_rate,
        # Robot description + TF
        joint_state_publisher_node,
        robot_state_publisher_node,
        camera_static_tf,
        # cuVSLAM
        cuvslam_node,
        # Nvblox mapper
        nvblox_mapper_node,
    ])
