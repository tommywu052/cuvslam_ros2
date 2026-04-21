#!/usr/bin/env python3
"""
Navigation launch using cuVSLAM visual SLAM instead of AMCL + LiDAR.

Dual-mode support:
  - RGBD mode (default): color + depth, no IMU
  - VIO mode (use_imu:=true): stereo IR + IMU (requires USB 3.x)

Requires (started externally):
  - RealSense camera node (with appropriate params for mode)
  - wl_base_node (wheel odometry)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    LogInfo, GroupAction
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
import os


def generate_launch_description():

    pkg_dir = os.path.expanduser(
        '~/legged_robot/ROS2_Packages/src/wheeled_legged_pkg')
    cuvslam_dir = os.path.expanduser('~/legged_robot/cuvslam_ros2')

    urdf_file = os.path.expanduser(
        '~/legged_robot/ROS2_Packages/src/wheel_legged_urdf_pkg/urdf/'
        'wheel_legged_urdf_pkg.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    nav2_params_file = os.path.join(cuvslam_dir, 'nav2_cuvslam_params.yaml')
    default_map_path = os.path.expanduser('~/maps/nv_16f_sa.yaml')

    # --- Launch arguments ---
    declare_map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to map yaml file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_file,
        description='Full path to Nav2 params file')

    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 (set true if GUI available)')

    # Camera mount offsets (base_link → camera_link)
    declare_cam_x = DeclareLaunchArgument('camera_x', default_value='0.135')
    declare_cam_y = DeclareLaunchArgument('camera_y', default_value='0.00')
    declare_cam_z = DeclareLaunchArgument('camera_z', default_value='0.054')

    # Initial pose in the occupancy grid map (align cuVSLAM to pre-built map)
    declare_init_x = DeclareLaunchArgument('initial_x', default_value='0.0')
    declare_init_y = DeclareLaunchArgument('initial_y', default_value='0.0')
    declare_init_yaw = DeclareLaunchArgument(
        'initial_yaw', default_value='0.0',
        description='Initial yaw in radians')

    # IMU mode toggle
    declare_use_imu = DeclareLaunchArgument(
        'use_imu', default_value='false',
        description='Enable VIO mode (stereo IR + IMU). Requires USB 3.x and realsense_vio_params.')

    # Relocalization
    declare_auto_localize = DeclareLaunchArgument(
        'auto_localize', default_value='true',
        description='Auto-localize in saved map on startup (skip manual initialpose)')
    declare_localize_radius = DeclareLaunchArgument(
        'localize_radius', default_value='3.0',
        description='Search radius (meters) for relocalization')

    # Nvblox (live 3D mapping + ESDF during navigation)
    declare_enable_nvblox = DeclareLaunchArgument(
        'enable_nvblox', default_value='true',
        description='Enable nvblox for live ESDF costmap and 3D mesh')

    declare_nvblox_map = DeclareLaunchArgument(
        'nvblox_map', default_value='',
        description='Path to .nvblox 3D map to preload (empty = fresh mapping)')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

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

    # base_link → camera_link static TF (URDF doesn't define camera)
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

    # --- cuVSLAM visual odometry (replaces AMCL) ---
    cuvslam_node = ExecuteProcess(
        cmd=['python3.10', os.path.join(cuvslam_dir, 'cuvslam_odom_node.py'),
             '--ros-args',
             '-p', 'enable_slam:=true',
             '-p', 'publish_tf:=true',
             '-p', ['use_imu:=', LaunchConfiguration('use_imu')],
             '-p', ['auto_localize:=', LaunchConfiguration('auto_localize')],
             '-p', ['localize_radius:=', LaunchConfiguration('localize_radius')],
             '-p', ['initial_x:=', LaunchConfiguration('initial_x')],
             '-p', ['initial_y:=', LaunchConfiguration('initial_y')],
             '-p', ['initial_yaw:=', LaunchConfiguration('initial_yaw')]],
        output='screen')

    # --- Nav2 stack (without AMCL) ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml_file}
        ])

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/cmd_vel')])

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file])

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file])

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file])

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file])

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ])

    # Lifecycle: only map_server (cuVSLAM handles localization, no AMCL)
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }])

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }])

    # --- Nvblox mapper (live ESDF costmap + 3D mesh during navigation) ---
    nvblox_map_path = LaunchConfiguration('nvblox_map')
    nvblox_mapper_node = ExecuteProcess(
        cmd=['python3.10', os.path.join(cuvslam_dir, 'nvblox_mapper_node.py'),
             '--ros-args',
             '-p', 'voxel_size:=0.05',
             '-p', 'max_integration_distance:=5.0',
             '-p', 'slice_height:=0.1',
             '-p', 'grid_publish_rate:=0.5',
             '-p', 'mesh_publish_rate:=0.5',
             '-p', 'esdf_publish_rate:=2.0',
             '-p', 'esdf_slice_min_height:=-0.1',
             '-p', 'esdf_slice_max_height:=1.5',
             '-p', ['load_map:=', nvblox_map_path]],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_nvblox')))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        # Launch args
        declare_map_arg,
        declare_params_file_arg,
        declare_rviz_arg,
        declare_cam_x,
        declare_cam_y,
        declare_cam_z,
        declare_init_x,
        declare_init_y,
        declare_init_yaw,
        declare_use_imu,
        declare_auto_localize,
        declare_localize_radius,
        declare_enable_nvblox,
        declare_nvblox_map,
        # Robot description + camera TF
        joint_state_publisher_node,
        robot_state_publisher_node,
        camera_static_tf,
        # cuVSLAM localization
        cuvslam_node,
        # Nav2 stack
        map_server_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        # Nvblox (live ESDF + mesh)
        nvblox_mapper_node,
        # Optional GUI
        rviz_node,
    ])
