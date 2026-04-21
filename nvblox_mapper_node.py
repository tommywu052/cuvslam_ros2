#!/usr/bin/env python3.10
"""
Nvblox Mapper ROS2 Node — GPU-accelerated 3D mapping for occupancy grid generation.

Subscribes to cuVSLAM pose (/visual_odom) and RealSense depth/color images,
integrates them into an nvblox TSDF voxel grid, and publishes a 2D occupancy
grid map (nav_msgs/OccupancyGrid) by slicing the TSDF at floor height.

This replaces LiDAR-based SLAM Toolbox for map building, achieving a
100% vision-only mapping pipeline: cuVSLAM (localization) + nvblox (3D map).

Usage:
  python3.10 nvblox_mapper_node.py --ros-args \
    -p voxel_size:=0.05 \
    -p max_integration_distance:=5.0 \
    -p slice_height:=0.1

Publishes:
  /nvblox/occupancy_grid   (nav_msgs/OccupancyGrid)       — 2D occupancy grid for Nav2
  /nvblox/color_mesh       (visualization_msgs/Marker)     — 3D color mesh for RViz
  /nvblox/esdf_pointcloud  (sensor_msgs/PointCloud2)       — 3D ESDF distance field for RViz
  /nvblox/esdf_costmap     (nav_msgs/OccupancyGrid)        — ESDF-based 2D costmap for Nav2
  /nvblox/map_status       (std_msgs/String)               — mapper status info

Services:
  /nvblox/save_map    (std_srvs/Trigger)  — save occupancy grid as .pgm/.yaml
  /nvblox/clear_map   (std_srvs/Trigger)  — clear the 3D map
"""

import os
import threading
import time
from typing import Optional

import numpy as np
import torch
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import struct
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Trigger
from std_msgs.msg import String, ColorRGBA, Header
from visualization_msgs.msg import Marker
import tf2_ros

from nvblox_torch.mapper import Mapper
from nvblox_torch.mapper_params import MapperParams, ProjectiveIntegratorParams
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
from nvblox_torch.sensor import Sensor


def odom_msg_to_4x4(odom_msg: Odometry) -> np.ndarray:
    """Convert nav_msgs/Odometry to 4x4 homogeneous transform (map -> base_link)."""
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    T = np.eye(4, dtype=np.float32)
    T[0, 3] = p.x
    T[1, 3] = p.y
    T[2, 3] = p.z
    # quaternion to rotation matrix (x, y, z, w)
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    T[0, 0] = 1 - 2 * (qy * qy + qz * qz)
    T[0, 1] = 2 * (qx * qy - qz * qw)
    T[0, 2] = 2 * (qx * qz + qy * qw)
    T[1, 0] = 2 * (qx * qy + qz * qw)
    T[1, 1] = 1 - 2 * (qx * qx + qz * qz)
    T[1, 2] = 2 * (qy * qz - qx * qw)
    T[2, 0] = 2 * (qx * qz - qy * qw)
    T[2, 1] = 2 * (qy * qz + qx * qw)
    T[2, 2] = 1 - 2 * (qx * qx + qy * qy)
    return T


def pose_changed_enough(A: np.ndarray, B: np.ndarray,
                        d_thresh: float, a_thresh: float) -> bool:
    """Return True when the translation or rotation between two 4x4 poses
    exceeds the given thresholds."""
    dt = np.linalg.norm(A[:3, 3] - B[:3, 3])
    if dt >= d_thresh:
        return True
    R_rel = A[:3, :3].T @ B[:3, :3]
    cos_angle = (np.trace(R_rel) - 1.0) / 2.0
    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
    return angle >= a_thresh


def ground_constrain_pose(T: np.ndarray) -> np.ndarray:
    """Project a 6-DOF pose down to ground-plane (x, y, yaw only).

    cuVSLAM RGBD mode without IMU drifts in pitch/roll. This strips the
    drifted components and keeps only the horizontal pose, ensuring the
    3D reconstruction stays aligned with the ground plane.
    """
    forward = T[:3, 0]
    yaw = np.arctan2(forward[1], forward[0])
    cy, sy = np.cos(yaw), np.sin(yaw)
    G = np.eye(4, dtype=np.float32)
    G[0, 0] = cy;  G[0, 1] = -sy
    G[1, 0] = sy;  G[1, 1] = cy
    G[0, 3] = T[0, 3]
    G[1, 3] = T[1, 3]
    # Z stays at 0 — base_link on the ground plane
    return G


class NvbloxMapperNode(Node):

    def __init__(self):
        super().__init__('nvblox_mapper_node')

        # --- Parameters ---
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_integration_distance', 5.0)
        self.declare_parameter('slice_height', 0.1)
        self.declare_parameter('slice_thickness', 0.3)
        self.declare_parameter('occupancy_threshold', -0.01)
        self.declare_parameter('free_threshold', 0.01)
        self.declare_parameter('grid_publish_rate', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('color_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('odom_topic', '/visual_odom')
        self.declare_parameter('save_dir', os.path.expanduser('~/maps'))
        self.declare_parameter('save_name', 'nvblox_map')
        self.declare_parameter('integrate_color', True)
        self.declare_parameter('mesh_publish_rate', 0.5)  # Hz, mesh update is heavy
        self.declare_parameter('esdf_publish_rate', 1.0)  # Hz
        self.declare_parameter('esdf_slice_min_height', -0.1)  # robot bottom
        self.declare_parameter('esdf_slice_max_height', 1.5)   # robot top
        self.declare_parameter('esdf_slice_num_heights', 5)    # sample heights
        self.declare_parameter('esdf_distance_threshold', 2.0) # max distance to visualize (m)
        self.declare_parameter('load_map', '')  # path to .nvblox file to preload

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_dist = self.get_parameter('max_integration_distance').value
        self.slice_height = self.get_parameter('slice_height').value
        self.slice_thickness = self.get_parameter('slice_thickness').value
        self.occ_thresh = self.get_parameter('occupancy_threshold').value
        self.free_thresh = self.get_parameter('free_threshold').value
        self.grid_rate = self.get_parameter('grid_publish_rate').value
        self.map_frame = self.get_parameter('map_frame').value
        self.save_dir = self.get_parameter('save_dir').value
        self.save_name = self.get_parameter('save_name').value
        self.integrate_color = self.get_parameter('integrate_color').value
        self.mesh_rate = self.get_parameter('mesh_publish_rate').value
        self.esdf_rate = self.get_parameter('esdf_publish_rate').value
        self.esdf_min_h = self.get_parameter('esdf_slice_min_height').value
        self.esdf_max_h = self.get_parameter('esdf_slice_max_height').value
        self.esdf_num_h = self.get_parameter('esdf_slice_num_heights').value
        self.esdf_dist_thresh = self.get_parameter('esdf_distance_threshold').value

        # --- State ---
        self.bridge = CvBridge()
        self.latest_odom: Optional[Odometry] = None
        self.odom_lock = threading.Lock()
        self.depth_sensor: Optional[Sensor] = None
        self.color_sensor: Optional[Sensor] = None
        self.camera_info_received = False
        self.frame_count = 0
        self.mapper: Optional[Mapper] = None
        self.base_T_camera: Optional[np.ndarray] = None

        # Keyframe gating — skip frames until the camera moves enough
        self.last_integrated_pose: Optional[np.ndarray] = None
        self.keyframe_dist_thresh = 0.05   # 5 cm
        self.keyframe_angle_thresh = 0.052  # ~3 degrees
        # Stationary detection — stop integrating when drift is the only "motion"
        self._raw_pose_history: list = []
        self._stationary = False
        self._stationary_check_window = 90   # ~3s at 30fps
        self._enter_stationary_thresh = 0.02   # 2cm → enter stationary mode
        self._exit_stationary_thresh = 0.15    # 15cm → exit (real walking ~50cm/s)
        self._motion_confirm_count = 0
        self._motion_confirm_needed = 15

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._init_nvblox_mapper()

        # Load pre-saved nvblox map if specified
        load_map_path = self.get_parameter('load_map').value
        if load_map_path and os.path.isfile(load_map_path):
            try:
                self.mapper.load_from_file(load_map_path, mapper_id=0)
                self.frame_count = 1  # mark as having data so mesh/ESDF publish immediately
                self.get_logger().info(f'Loaded nvblox map: {load_map_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load nvblox map: {e}')
        elif load_map_path:
            self.get_logger().warn(f'Nvblox map file not found: {load_map_path}')

        # --- Subscribers ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10)

        depth_topic = self.get_parameter('depth_topic').value
        color_topic = self.get_parameter('color_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
        color_info_topic = self.get_parameter('color_info_topic').value

        self.depth_info_sub = self.create_subscription(
            CameraInfo, depth_info_topic, self._depth_info_callback, sensor_qos)
        self.color_info_sub = self.create_subscription(
            CameraInfo, color_info_topic, self._color_info_callback, sensor_qos)

        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=sensor_qos)
        self.color_sub = Subscriber(self, Image, color_topic, qos_profile=sensor_qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self._image_callback)

        # --- Publishers ---
        self.grid_pub = self.create_publisher(OccupancyGrid, '/nvblox/occupancy_grid', 10)
        self.mesh_pub = self.create_publisher(Marker, '/nvblox/color_mesh', 1)
        self.esdf_pc_pub = self.create_publisher(PointCloud2, '/nvblox/esdf_pointcloud', 1)
        self.esdf_costmap_pub = self.create_publisher(OccupancyGrid, '/nvblox/esdf_costmap', 10)
        self.status_pub = self.create_publisher(String, '/nvblox/map_status', 10)

        # --- Services ---
        self.create_service(Trigger, '/nvblox/save_map', self._save_map_callback)
        self.create_service(Trigger, '/nvblox/clear_map', self._clear_map_callback)

        # --- Timers ---
        if self.grid_rate > 0:
            self.create_timer(1.0 / self.grid_rate, self._publish_occupancy_grid)
        if self.mesh_rate > 0:
            self.create_timer(1.0 / self.mesh_rate, self._publish_color_mesh)
        if self.esdf_rate > 0:
            self.create_timer(1.0 / self.esdf_rate, self._publish_esdf)

        self.get_logger().info(
            f'Nvblox mapper initialized: voxel={self.voxel_size}m, '
            f'max_dist={self.max_dist}m, slice_height={self.slice_height}m')

    def _init_nvblox_mapper(self):
        """Initialize the nvblox Mapper with TSDF integrator."""
        proj_params = ProjectiveIntegratorParams()
        proj_params.projective_integrator_max_integration_distance_m = self.max_dist
        mapper_params = MapperParams()
        mapper_params.set_projective_integrator_params(proj_params)

        self.mapper = Mapper(
            voxel_sizes_m=self.voxel_size,
            integrator_types=ProjectiveIntegratorType.TSDF,
            mapper_parameters=mapper_params)
        self.get_logger().info('nvblox Mapper created (TSDF mode)')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        with self.odom_lock:
            self.latest_odom = msg

    def _depth_info_callback(self, msg: CameraInfo):
        if self.depth_sensor is not None:
            return
        self.depth_sensor = Sensor.from_camera(
            fu=msg.k[0], fv=msg.k[4], cu=msg.k[2], cv=msg.k[5],
            width=msg.width, height=msg.height)
        self.get_logger().info(
            f'Depth sensor: {msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}')
        self._check_ready()

    def _color_info_callback(self, msg: CameraInfo):
        if self.color_sensor is not None:
            return
        self.color_sensor = Sensor.from_camera(
            fu=msg.k[0], fv=msg.k[4], cu=msg.k[2], cv=msg.k[5],
            width=msg.width, height=msg.height)
        self.get_logger().info(
            f'Color sensor: {msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}')
        self._check_ready()

    def _check_ready(self):
        if self.depth_sensor is not None and self.color_sensor is not None:
            if not self.camera_info_received:
                self.camera_info_received = True
                self.get_logger().info('Camera info received — ready to map')

    def _lookup_base_to_camera(self) -> bool:
        """Look up base_link → camera_depth_optical_frame transform via TF."""
        if self.base_T_camera is not None:
            return True
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'base_link', 'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            T = np.eye(4, dtype=np.float32)
            T[0, 3] = t.x
            T[1, 3] = t.y
            T[2, 3] = t.z
            qx, qy, qz, qw = r.x, r.y, r.z, r.w
            T[0, 0] = 1 - 2 * (qy * qy + qz * qz)
            T[0, 1] = 2 * (qx * qy - qz * qw)
            T[0, 2] = 2 * (qx * qz + qy * qw)
            T[1, 0] = 2 * (qx * qy + qz * qw)
            T[1, 1] = 1 - 2 * (qx * qx + qz * qz)
            T[1, 2] = 2 * (qy * qz - qx * qw)
            T[2, 0] = 2 * (qx * qz - qy * qw)
            T[2, 1] = 2 * (qy * qz + qx * qw)
            T[2, 2] = 1 - 2 * (qx * qx + qy * qy)
            self.base_T_camera = T
            self.get_logger().info(
                f'base_link → camera_depth_optical_frame: '
                f't=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]')
            return True
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup base→camera failed: {e}', throttle_duration_sec=5.0)
            return False

    def _image_callback(self, depth_msg: Image, color_msg: Image):
        """Process synchronized depth + color frames."""
        if self.depth_sensor is None or not self._lookup_base_to_camera():
            return

        with self.odom_lock:
            odom = self.latest_odom
        if odom is None:
            return

        # map_T_base from cuVSLAM odometry — apply ground constraint to
        # eliminate pitch/roll drift (cuVSLAM RGBD has no gravity reference)
        map_T_base_raw = odom_msg_to_4x4(odom)
        map_T_base = ground_constrain_pose(map_T_base_raw)
        map_T_camera = map_T_base @ self.base_T_camera

        # --- Stationary detection ---
        # Track raw constrained positions; if the spread over a sliding window
        # is tiny, the "motion" is just drift and we should stop integrating.
        pos = map_T_camera[:3, 3].copy()
        self._raw_pose_history.append(pos)
        if len(self._raw_pose_history) > self._stationary_check_window:
            self._raw_pose_history.pop(0)
        if len(self._raw_pose_history) >= self._stationary_check_window:
            pts = np.array(self._raw_pose_history)
            spread = float(np.max(pts.max(axis=0) - pts.min(axis=0)))

            if self._stationary:
                # Require large, sustained spread to resume (hysteresis)
                if spread >= self._exit_stationary_thresh:
                    self._motion_confirm_count += 1
                    if self._motion_confirm_count >= self._motion_confirm_needed:
                        self._stationary = False
                        self._motion_confirm_count = 0
                        self.get_logger().info(
                            f'Motion confirmed (spread={spread:.4f}m) — resuming')
                else:
                    self._motion_confirm_count = 0
            else:
                if spread < self._enter_stationary_thresh:
                    self._stationary = True
                    self._motion_confirm_count = 0
                    self.get_logger().info(
                        f'Stationary (spread={spread:.4f}m) — pausing integration')

        # Skip integration when stationary (drift-only) or below keyframe threshold
        if self._stationary and self.frame_count > 0:
            return
        if self.last_integrated_pose is not None and not pose_changed_enough(
                self.last_integrated_pose, map_T_camera,
                self.keyframe_dist_thresh, self.keyframe_angle_thresh):
            return

        try:
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_np = depth_cv.astype(np.float32) / 1000.0
            depth_np[depth_np <= 0] = 0.0
            depth_tensor = torch.from_numpy(depth_np).cuda()

            pose_tensor = torch.from_numpy(map_T_camera).float()

            self.mapper.add_depth_frame(depth_tensor, pose_tensor, self.depth_sensor)

            if self.integrate_color and self.color_sensor is not None:
                color_cv = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
                color_tensor = torch.from_numpy(color_cv).cuda()
                self.mapper.add_color_frame(
                    color_tensor, pose_tensor, self.color_sensor)

            self.last_integrated_pose = map_T_camera.copy()
            self.frame_count += 1
            if self.frame_count == 1:
                cam_z = map_T_camera[:3, 2]
                cam_y = map_T_camera[:3, 1]
                self.get_logger().info(
                    f'First frame map_T_camera: '
                    f'cam_Z(depth)=[{cam_z[0]:.3f},{cam_z[1]:.3f},{cam_z[2]:.3f}] '
                    f'cam_Y(down)=[{cam_y[0]:.3f},{cam_y[1]:.3f},{cam_y[2]:.3f}]')
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Integrated {self.frame_count} frames')

        except Exception as e:
            self.get_logger().error(f'Frame integration error: {e}')

    # ------------------------------------------------------------------
    # Occupancy Grid Generation (TSDF → 2D slice)
    # ------------------------------------------------------------------

    def _publish_occupancy_grid(self):
        """Generate and publish 2D occupancy grid from TSDF slice at floor height."""
        if self.mapper is None or self.frame_count == 0:
            return

        try:
            tsdf_layer = self.mapper.tsdf_layer_view(mapper_id=0)
            if tsdf_layer.num_blocks() == 0:
                return

            # Get all block indices to determine map extents
            block_indices = tsdf_layer.get_all_block_indices()
            if block_indices.shape[0] == 0:
                return

            voxel_size = tsdf_layer.voxel_size()
            block_size = 8  # voxels per block dimension

            # Determine AABB in block indices
            min_idx, _ = block_indices.min(dim=0)
            max_idx, _ = block_indices.max(dim=0)

            # Convert slice height range to Z block indices
            z_min_voxel = int(np.floor(
                (self.slice_height - self.slice_thickness / 2.0) / (voxel_size * block_size)))
            z_max_voxel = int(np.ceil(
                (self.slice_height + self.slice_thickness / 2.0) / (voxel_size * block_size)))

            # Iterate over XY blocks, query TSDF at slice height
            x_blocks = max_idx[0].item() - min_idx[0].item() + 1
            y_blocks = max_idx[1].item() - min_idx[1].item() + 1
            x_voxels = x_blocks * block_size
            y_voxels = y_blocks * block_size

            if x_voxels <= 0 or y_voxels <= 0 or x_voxels > 2000 or y_voxels > 2000:
                return

            # Build query points for the XY slice at slice_height
            x_min_m = (min_idx[0].item() * block_size + 0.5) * voxel_size
            y_min_m = (min_idx[1].item() * block_size + 0.5) * voxel_size

            x_coords = torch.arange(x_voxels, device='cuda') * voxel_size + x_min_m
            y_coords = torch.arange(y_voxels, device='cuda') * voxel_size + y_min_m
            xx, yy = torch.meshgrid(x_coords, y_coords, indexing='ij')
            zz = torch.full_like(xx, self.slice_height)

            query_points = torch.stack([xx, yy, zz], dim=-1).reshape(-1, 3)

            # Query TSDF at these points
            from nvblox_torch.mapper import QueryType
            result = self.mapper.query_layer(
                QueryType.TSDF, query_points, mapper_id=0)

            tsdf_distances = result[:, 0]
            tsdf_weights = result[:, 1]

            # Classify: occupied / free / unknown
            grid_data = np.full(x_voxels * y_voxels, -1, dtype=np.int8)

            observed = tsdf_weights.cpu().numpy() > 0.5
            distances = tsdf_distances.cpu().numpy()

            occupied = observed & (distances < self.occ_thresh)
            free = observed & (distances > self.free_thresh)

            grid_data[occupied] = 100  # occupied
            grid_data[free] = 0       # free

            # Reshape to 2D (nvblox uses XYZ, OccupancyGrid uses row-major with Y as rows)
            grid_2d = grid_data.reshape(x_voxels, y_voxels)
            # OccupancyGrid: data[y * width + x], row-major, y increasing upward
            # Transpose so that X→columns, Y→rows, then flip Y for ROS convention
            grid_ros = grid_2d.T  # shape: (y_voxels, x_voxels)

            # Build OccupancyGrid message
            occ_grid = OccupancyGrid()
            occ_grid.header.stamp = self.get_clock().now().to_msg()
            occ_grid.header.frame_id = self.map_frame
            occ_grid.info = MapMetaData()
            occ_grid.info.resolution = float(voxel_size)
            occ_grid.info.width = x_voxels
            occ_grid.info.height = y_voxels
            occ_grid.info.origin = Pose(
                position=Point(x=float(x_min_m - voxel_size * 0.5),
                               y=float(y_min_m - voxel_size * 0.5),
                               z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            occ_grid.data = grid_ros.flatten().tolist()

            self.grid_pub.publish(occ_grid)

            # Publish status
            status = String()
            n_obs = int(observed.sum())
            n_occ = int(occupied.sum())
            n_free = int(free.sum())
            status.data = (
                f'frames={self.frame_count}, blocks={tsdf_layer.num_blocks()}, '
                f'grid={x_voxels}x{y_voxels}, observed={n_obs}, '
                f'occupied={n_occ}, free={n_free}')
            self.status_pub.publish(status)

        except Exception as e:
            self.get_logger().error(f'Occupancy grid generation error: {e}')

    # ------------------------------------------------------------------
    # 3D Color Mesh Visualization
    # ------------------------------------------------------------------

    def _publish_color_mesh(self):
        """Extract color mesh from nvblox and publish as Marker for RViz."""
        if self.mapper is None or self.frame_count < 1:
            return

        try:
            self.mapper.update_color_mesh(mapper_id=0)
            color_mesh = self.mapper.get_color_mesh(mapper_id=0)

            verts = color_mesh.vertices().cpu()       # (N, 3) float
            colors = color_mesh.vertex_colors().cpu()  # (N, 3) or (N, 4) uint8
            tris = color_mesh.triangles().cpu()        # (M, 3) int

            n_verts = verts.shape[0]
            n_tris = tris.shape[0]
            if n_verts == 0 or n_tris == 0:
                return

            MAX_TRIANGLES = 500000
            if n_tris > MAX_TRIANGLES:
                tris = tris[:MAX_TRIANGLES]
                n_tris = MAX_TRIANGLES

            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self.map_frame
            marker.ns = 'nvblox_mesh'
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            verts_np = verts.numpy()
            colors_np = colors.numpy().astype(np.float32)
            if colors_np.max() > 1.0:
                colors_np = colors_np / 255.0
            n_color_channels = colors_np.shape[1] if colors_np.ndim > 1 else 1

            tris_flat = tris.numpy().flatten()
            tri_verts = verts_np[tris_flat]      # (M*3, 3)
            tri_colors = colors_np[tris_flat]    # (M*3, C)

            points = []
            marker_colors = []
            for i in range(tri_verts.shape[0]):
                points.append(Point(
                    x=float(tri_verts[i, 0]),
                    y=float(tri_verts[i, 1]),
                    z=float(tri_verts[i, 2])))
                r = float(tri_colors[i, 0]) if n_color_channels >= 1 else 0.5
                g = float(tri_colors[i, 1]) if n_color_channels >= 2 else 0.5
                b = float(tri_colors[i, 2]) if n_color_channels >= 3 else 0.5
                a = float(tri_colors[i, 3]) if n_color_channels >= 4 else 1.0
                marker_colors.append(ColorRGBA(r=r, g=g, b=b, a=a))

            marker.points = points
            marker.colors = marker_colors
            self.mesh_pub.publish(marker)

            if self.frame_count % 50 == 0:
                self.get_logger().info(
                    f'Published color mesh: {n_verts} vertices, {n_tris} triangles')

        except Exception as e:
            self.get_logger().error(
                f'Color mesh publish error: {e}', throttle_duration_sec=10.0)

    # ------------------------------------------------------------------
    # ESDF (Euclidean Signed Distance Field)
    # ------------------------------------------------------------------

    def _publish_esdf(self):
        """Compute ESDF from TSDF, publish 3D pointcloud and 2D costmap."""
        if self.mapper is None or self.frame_count < 1:
            return

        try:
            self.mapper.update_esdf(mapper_id=0)

            tsdf_layer = self.mapper.tsdf_layer_view(mapper_id=0)
            if tsdf_layer.num_blocks() == 0:
                return

            block_indices = tsdf_layer.get_all_block_indices()
            if block_indices.shape[0] == 0:
                return

            voxel_size = tsdf_layer.voxel_size()
            block_size = 8

            min_idx, _ = block_indices.min(dim=0)
            max_idx, _ = block_indices.max(dim=0)

            x_blocks = max_idx[0].item() - min_idx[0].item() + 1
            y_blocks = max_idx[1].item() - min_idx[1].item() + 1
            x_voxels = x_blocks * block_size
            y_voxels = y_blocks * block_size

            if x_voxels <= 0 or y_voxels <= 0 or x_voxels > 2000 or y_voxels > 2000:
                return

            x_min_m = (min_idx[0].item() * block_size + 0.5) * voxel_size
            y_min_m = (min_idx[1].item() * block_size + 0.5) * voxel_size

            x_coords = torch.arange(x_voxels, device='cuda') * voxel_size + x_min_m
            y_coords = torch.arange(y_voxels, device='cuda') * voxel_size + y_min_m

            from nvblox_torch.mapper import QueryType, constants
            esdf_unknown = constants.esdf_unknown_distance()

            heights = np.linspace(self.esdf_min_h, self.esdf_max_h, self.esdf_num_h)
            all_points = []
            all_distances = []
            min_distances = None

            for h in heights:
                xx, yy = torch.meshgrid(x_coords, y_coords, indexing='ij')
                zz = torch.full_like(xx, float(h))
                rr = torch.zeros_like(xx)
                query = torch.stack([xx, yy, zz, rr], dim=-1).reshape(-1, 4)

                result = self.mapper.query_layer(QueryType.ESDF, query, mapper_id=0)
                dists = result[:, 0]

                dists_np = dists.cpu().numpy()

                # For pointcloud: filter valid points
                valid = (dists_np != esdf_unknown) & (np.abs(dists_np) < self.esdf_dist_thresh)
                if valid.sum() > 0:
                    valid_pts = query[valid, :3].cpu().numpy()
                    all_points.append(valid_pts)
                    all_distances.append(dists_np[valid])

                # For costmap: accumulate min distance per XY cell
                dists_2d = dists_np.reshape(x_voxels, y_voxels)
                if min_distances is None:
                    min_distances = dists_2d.copy()
                else:
                    min_distances = np.minimum(min_distances, dists_2d)

            # Publish 3D pointcloud
            if all_points:
                pts = np.concatenate(all_points, axis=0)
                dists_concat = np.concatenate(all_distances, axis=0)
                self._publish_esdf_pointcloud(pts, dists_concat)

            # Publish 2D costmap
            if min_distances is not None:
                self._publish_esdf_costmap_from_distances(
                    min_distances, x_voxels, y_voxels,
                    x_min_m, y_min_m, voxel_size, esdf_unknown)

        except Exception as e:
            self.get_logger().error(
                f'ESDF publish error: {e}', throttle_duration_sec=10.0)

    def _publish_esdf_pointcloud(self, points: np.ndarray, distances: np.ndarray):
        """Publish ESDF as PointCloud2 with distance-based coloring for RViz.

        Fields: x, y, z, distance (float32). Use 'distance' for color in RViz.
        """
        n = points.shape[0]
        if n == 0:
            return

        MAX_ESDF_POINTS = 500000
        if n > MAX_ESDF_POINTS:
            idx = np.random.choice(n, MAX_ESDF_POINTS, replace=False)
            points = points[idx]
            distances = distances[idx]
            n = MAX_ESDF_POINTS

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 16  # 4 floats x 4 bytes
        data = bytearray(n * point_step)
        for i in range(n):
            struct.pack_into('ffff', data, i * point_step,
                             float(points[i, 0]), float(points[i, 1]),
                             float(points[i, 2]), float(distances[i]))

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n
        msg.data = bytes(data)
        msg.is_dense = True

        self.esdf_pc_pub.publish(msg)

    def _publish_esdf_costmap_from_distances(self, min_distances,
                                             x_voxels, y_voxels,
                                             x_min_m, y_min_m, voxel_size,
                                             esdf_unknown):
        """Publish 2D costmap from pre-computed min ESDF distances.

        Each cell = min ESDF distance across sampled heights at that XY position.
        Cells with small distance -> high cost (obstacle nearby in 3D).
        """
        obstacle_thresh = 0.15  # cells closer than 15cm to surface = obstacle
        grid_data = np.full(min_distances.shape, -1, dtype=np.int8)

        observed = np.abs(min_distances - esdf_unknown) > 1.0
        if not np.any(observed):
            return

        dist = min_distances[observed]

        costs = np.zeros_like(dist, dtype=np.int8)
        costs[dist <= obstacle_thresh] = 100   # obstacle
        costs[dist > obstacle_thresh] = 0      # free

        grid_data[observed] = costs

        grid_ros = grid_data.T

        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = self.map_frame
        occ_grid.info = MapMetaData()
        occ_grid.info.resolution = float(voxel_size)
        occ_grid.info.width = x_voxels
        occ_grid.info.height = y_voxels
        occ_grid.info.origin = Pose(
            position=Point(x=float(x_min_m - voxel_size * 0.5),
                           y=float(y_min_m - voxel_size * 0.5),
                           z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        occ_grid.data = grid_ros.flatten().tolist()

        self.esdf_costmap_pub.publish(occ_grid)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _save_map_callback(self, request, response):
        """Save the current occupancy grid as a PGM + YAML file pair (Nav2 compatible)."""
        try:
            os.makedirs(self.save_dir, exist_ok=True)

            tsdf_layer = self.mapper.tsdf_layer_view(mapper_id=0)
            if tsdf_layer.num_blocks() == 0:
                response.success = False
                response.message = 'No map data to save (0 blocks)'
                return response

            block_indices = tsdf_layer.get_all_block_indices()
            voxel_size = tsdf_layer.voxel_size()
            block_size = 8

            min_idx, _ = block_indices.min(dim=0)
            max_idx, _ = block_indices.max(dim=0)

            x_blocks = max_idx[0].item() - min_idx[0].item() + 1
            y_blocks = max_idx[1].item() - min_idx[1].item() + 1
            x_voxels = x_blocks * block_size
            y_voxels = y_blocks * block_size

            x_min_m = (min_idx[0].item() * block_size + 0.5) * voxel_size
            y_min_m = (min_idx[1].item() * block_size + 0.5) * voxel_size

            x_coords = torch.arange(x_voxels, device='cuda') * voxel_size + x_min_m
            y_coords = torch.arange(y_voxels, device='cuda') * voxel_size + y_min_m
            xx, yy = torch.meshgrid(x_coords, y_coords, indexing='ij')
            zz = torch.full_like(xx, self.slice_height)
            query_points = torch.stack([xx, yy, zz], dim=-1).reshape(-1, 3)

            from nvblox_torch.mapper import QueryType
            result = self.mapper.query_layer(
                QueryType.TSDF, query_points, mapper_id=0)

            distances = result[:, 0].cpu().numpy()
            weights = result[:, 1].cpu().numpy()
            observed = weights > 0.5

            # PGM pixel values: 254=free, 0=occupied, 205=unknown
            pgm_data = np.full(x_voxels * y_voxels, 205, dtype=np.uint8)
            pgm_data[observed & (distances > self.free_thresh)] = 254
            pgm_data[observed & (distances < self.occ_thresh)] = 0

            grid_2d = pgm_data.reshape(x_voxels, y_voxels).T
            # Flip vertically for PGM (top=max Y)
            grid_2d = np.flipud(grid_2d)

            pgm_path = os.path.join(self.save_dir, f'{self.save_name}.pgm')
            yaml_path = os.path.join(self.save_dir, f'{self.save_name}.yaml')

            # Write PGM (P5 binary)
            h, w = grid_2d.shape
            with open(pgm_path, 'wb') as f:
                f.write(f'P5\n{w} {h}\n255\n'.encode())
                f.write(grid_2d.tobytes())

            # Write YAML (Nav2 map_server format)
            map_yaml = {
                'image': f'{self.save_name}.pgm',
                'resolution': float(voxel_size),
                'origin': [
                    float(x_min_m - voxel_size * 0.5),
                    float(y_min_m - voxel_size * 0.5),
                    0.0
                ],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
                'mode': 'trinary',
            }
            with open(yaml_path, 'w') as f:
                yaml.dump(map_yaml, f, default_flow_style=False)

            # Also save nvblox native map
            blox_path = os.path.join(self.save_dir, f'{self.save_name}.nvblox')
            self.mapper.save_map(blox_path, mapper_id=0)

            response.success = True
            response.message = (
                f'Map saved: {pgm_path} ({w}x{h}), {yaml_path}, {blox_path}')
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Save failed: {e}'
            self.get_logger().error(response.message)
        return response

    def _clear_map_callback(self, request, response):
        """Clear the nvblox map."""
        try:
            self.mapper.clear(mapper_id=0)
            self.frame_count = 0
            response.success = True
            response.message = 'Map cleared'
            self.get_logger().info('Map cleared')
        except Exception as e:
            response.success = False
            response.message = f'Clear failed: {e}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NvbloxMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
