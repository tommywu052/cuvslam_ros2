#!/usr/bin/env python3.10
"""
cuVSLAM Visual Odometry ROS2 Node for wheeled-legged robot.

Dual-mode support:
  - RGBD mode (default): color + aligned depth, no IMU
  - VIO mode (use_imu=True): stereo IR + IMU (Inertial odometry)

Publishes:
  - /visual_odom (nav_msgs/Odometry)
  - TF: map -> odom (replacing AMCL for global localization)

Usage:
  python3.10 cuvslam_odom_node.py --ros-args -p use_imu:=false   # RGBD mode
  python3.10 cuvslam_odom_node.py --ros-args -p use_imu:=true    # VIO mode
"""

import collections
import json
import os
import threading
import time
from typing import Optional, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    TransformStamped, PoseWithCovariance, TwistWithCovariance,
    Quaternion, Vector3
)
from std_srvs.srv import Trigger, SetBool
import tf2_ros
from tf2_ros import TransformException

import cuvslam as vslam

# IMU noise parameters for RealSense D435I (conservative defaults)
IMU_GYROSCOPE_NOISE_DENSITY = 6.0673370376614875e-03
IMU_GYROSCOPE_RANDOM_WALK = 3.6211951458325785e-05
IMU_ACCELEROMETER_NOISE_DENSITY = 3.3621979208052800e-02
IMU_ACCELEROMETER_RANDOM_WALK = 9.8256589971851467e-04
IMU_FREQUENCY = 200


# cuVSLAM optical frame (OpenCV) to ROS base_link convention
# OpenCV: X-right, Y-down, Z-forward
# ROS:    X-forward, Y-left, Z-up
T_ROS_FROM_OPTICAL = np.array([
    [0,  0,  1, 0],
    [-1, 0,  0, 0],
    [0, -1,  0, 0],
    [0,  0,  0, 1]
], dtype=np.float64)


def quat_to_matrix(q):
    """Convert quaternion [x,y,z,w] to 4x4 homogeneous matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),     0],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),     0],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y), 0],
        [0,                 0,                  0,                  1]
    ], dtype=np.float64)


def matrix_to_quat(m):
    """Convert 3x3 or 4x4 matrix to quaternion [x,y,z,w]."""
    r = m[:3, :3]
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def pose_to_matrix(pose: vslam.Pose) -> np.ndarray:
    """Convert cuVSLAM Pose to 4x4 homogeneous matrix."""
    q = pose.rotation  # [x, y, z, w]
    t = pose.translation  # [x, y, z]
    mat = quat_to_matrix(q)
    mat[0, 3] = t[0]
    mat[1, 3] = t[1]
    mat[2, 3] = t[2]
    return mat


def tf_to_matrix(tf_msg: TransformStamped) -> np.ndarray:
    """Convert ROS2 TransformStamped to 4x4 matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    mat = quat_to_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat


class CuvslamOdomNode(Node):
    def __init__(self):
        super().__init__('cuvslam_odom_node')

        # --- Common parameters ---
        self.declare_parameter('use_imu', False)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('enable_slam', True)
        self.declare_parameter('map_folder', '/home/robotester1/legged_robot/cuvslam_ros2/maps')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('warmup_frames', 30)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        # --- Relocalization parameters ---
        self.declare_parameter('auto_localize', True)
        self.declare_parameter('localize_radius', 3.0)
        self.declare_parameter('localize_angular_step', 0.3)

        # --- RGBD mode parameters ---
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')

        # --- VIO mode parameters ---
        self.declare_parameter('infra1_topic', '/camera/camera/infra1/image_rect_raw')
        self.declare_parameter('infra2_topic', '/camera/camera/infra2/image_rect_raw')
        self.declare_parameter('infra1_info_topic', '/camera/camera/infra1/camera_info')
        self.declare_parameter('infra2_info_topic', '/camera/camera/infra2/camera_info')
        self.declare_parameter('imu_topic', '/camera/camera/imu')
        self.declare_parameter('infra1_optical_frame', 'camera_infra1_optical_frame')

        # Read parameters
        self.use_imu = self.get_parameter('use_imu').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.enable_slam = self.get_parameter('enable_slam').value
        self.map_folder = self.get_parameter('map_folder').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.warmup_frames = self.get_parameter('warmup_frames').value
        self._odom_base_fallback = True  # assume base offline until proven otherwise

        # Camera optical frame depends on mode
        if self.use_imu:
            self.camera_optical_frame = self.get_parameter('infra1_optical_frame').value
        else:
            self.camera_optical_frame = self.get_parameter('camera_optical_frame').value

        # Build initial pose matrix (2D: x, y, yaw)
        ix = self.get_parameter('initial_x').value
        iy = self.get_parameter('initial_y').value
        iyaw = self.get_parameter('initial_yaw').value
        c, s = np.cos(iyaw), np.sin(iyaw)
        self.initial_pose = np.array([
            [c, -s, 0, ix],
            [s,  c, 0, iy],
            [0,  0, 1,  0],
            [0,  0, 0,  1]
        ], dtype=np.float64)
        if ix != 0.0 or iy != 0.0 or iyaw != 0.0:
            self.get_logger().info(
                f'Initial pose: x={ix:.2f}, y={iy:.2f}, yaw={np.degrees(iyaw):.1f}°')

        self.bridge = CvBridge()
        self.tracker: Optional[vslam.Tracker] = None
        self.frame_count = 0
        self.tracking_lost = False
        self.prev_pose_matrix: Optional[np.ndarray] = None
        self.prev_timestamp_ns: Optional[int] = None

        # VIO mode: IMU buffer and lock
        self._imu_lock = threading.Lock()
        self._imu_buffer: collections.deque = collections.deque(maxlen=2000)
        self._last_registered_imu_ts: int = 0

        # Relocalization state
        self._relocalized = False
        self._localization_attempted = False
        self._localization_in_progress = False
        self._last_images_for_localize: Optional[List[np.ndarray]] = None
        self.auto_localize = self.get_parameter('auto_localize').value
        self.localize_radius = self.get_parameter('localize_radius').value
        self.localize_angular_step = self.get_parameter('localize_angular_step').value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.base_T_camera: Optional[np.ndarray] = None

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)

        # Services
        self.save_map_srv = self.create_service(
            Trigger, '/cuvslam/save_map', self.save_map_callback)
        self.reset_srv = self.create_service(
            Trigger, '/cuvslam/reset', self.reset_callback)
        self.localize_srv = self.create_service(
            Trigger, '/cuvslam/localize', self._localize_service_callback)

        # Subscribe to /initialpose (from RViz "2D Pose Estimate" tool)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self._initialpose_callback, 10)

        # Mode-specific initialization
        mode_str = 'VIO (stereo IR + IMU)' if self.use_imu else 'RGBD (color + depth)'
        self.get_logger().info(f'cuVSLAM node started in {mode_str} mode')
        self.get_logger().info(f'cuVSLAM version: {vslam.get_version()[0]}')

        if self.use_imu:
            self._init_vio_mode()
        else:
            self._init_rgbd_mode()

    # ------------------------------------------------------------------
    # Initialization helpers
    # ------------------------------------------------------------------

    def _init_rgbd_mode(self):
        """Initialize subscriptions for RGBD mode."""
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic,
            self._rgbd_camera_info_callback, 10)
        self.get_logger().info(f'Waiting for camera_info on {camera_info_topic}...')

    def _init_vio_mode(self):
        """Initialize subscriptions for VIO mode (stereo IR + IMU)."""
        infra1_info_topic = self.get_parameter('infra1_info_topic').value
        infra2_info_topic = self.get_parameter('infra2_info_topic').value
        self._vio_infra1_info: Optional[CameraInfo] = None
        self._vio_infra2_info: Optional[CameraInfo] = None

        self._infra1_info_sub = self.create_subscription(
            CameraInfo, infra1_info_topic,
            self._vio_infra1_info_callback, 10)
        self._infra2_info_sub = self.create_subscription(
            CameraInfo, infra2_info_topic,
            self._vio_infra2_info_callback, 10)
        self.get_logger().info(
            f'VIO: Waiting for camera_info on {infra1_info_topic} and {infra2_info_topic}...')

    # ------------------------------------------------------------------
    # RGBD mode: camera info and image subscribers
    # ------------------------------------------------------------------

    def _rgbd_camera_info_callback(self, msg: CameraInfo):
        """Initialize cuVSLAM tracker from camera intrinsics (RGBD mode, called once)."""
        if self.tracker is not None:
            return

        self.get_logger().info(
            f'Received camera_info: {msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}')

        cam = vslam.Camera()
        cam.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
        cam.size = (msg.width, msg.height)
        cam.focal = (msg.k[0], msg.k[4])
        cam.principal = (msg.k[2], msg.k[5])

        rig = vslam.Rig()
        rig.cameras = [cam]

        rgbd_settings = vslam.Tracker.OdometryRGBDSettings()
        rgbd_settings.depth_scale_factor = 1000.0
        rgbd_settings.depth_camera_id = 0
        rgbd_settings.enable_depth_stereo_tracking = False

        odom_cfg = vslam.Tracker.OdometryConfig(
            async_sba=True,
            enable_final_landmarks_export=True,
            odometry_mode=vslam.Tracker.OdometryMode.RGBD,
            rgbd_settings=rgbd_settings
        )

        slam_cfg = None
        if self.enable_slam:
            slam_cfg = vslam.Tracker.SlamConfig(sync_mode=False)
            self.get_logger().info('SLAM enabled with async mode')

        self.tracker = vslam.Tracker(rig, odom_cfg, slam_cfg)
        self.get_logger().info('cuVSLAM Tracker initialized (RGBD mode)')

        self.destroy_subscription(self.camera_info_sub)
        self._lookup_camera_tf()

    def _start_rgbd_subscribers(self):
        """Start image subscribers for RGBD mode."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.color_sub = Subscriber(self, Image, self.color_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self._rgbd_image_callback)
        self.get_logger().info(
            f'RGBD subscribed: {self.color_topic} + {self.depth_topic}')

    # ------------------------------------------------------------------
    # VIO mode: camera info, IMU, and image subscribers
    # ------------------------------------------------------------------

    def _vio_infra1_info_callback(self, msg: CameraInfo):
        """Receive infra1 camera info (VIO mode)."""
        if self.tracker is not None:
            return
        self._vio_infra1_info = msg
        self._try_init_vio_tracker()

    def _vio_infra2_info_callback(self, msg: CameraInfo):
        """Receive infra2 camera info (VIO mode)."""
        if self.tracker is not None:
            return
        self._vio_infra2_info = msg
        self._try_init_vio_tracker()

    def _try_init_vio_tracker(self):
        """Initialize VIO tracker once both camera infos are received."""
        if self.tracker is not None:
            return
        if self._vio_infra1_info is None or self._vio_infra2_info is None:
            return

        info1 = self._vio_infra1_info
        info2 = self._vio_infra2_info

        self.get_logger().info(
            f'VIO: infra1 {info1.width}x{info1.height} fx={info1.k[0]:.1f}, '
            f'infra2 {info2.width}x{info2.height} fx={info2.k[0]:.1f}')

        # Left camera (infra1) - primary, identity pose
        cam_left = vslam.Camera()
        cam_left.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
        cam_left.size = (info1.width, info1.height)
        cam_left.focal = (info1.k[0], info1.k[4])
        cam_left.principal = (info1.k[2], info1.k[5])

        # Right camera (infra2) - get relative pose from TF
        cam_right = vslam.Camera()
        cam_right.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
        cam_right.size = (info2.width, info2.height)
        cam_right.focal = (info2.k[0], info2.k[4])
        cam_right.principal = (info2.k[2], info2.k[5])

        # Stereo baseline: infra2 relative to infra1
        # D435I typical baseline is ~50mm (0.05m) in X
        # We get this from the camera_info P matrix of infra2:
        # P[3] = -fx * baseline for the right camera
        baseline = 0.05  # default fallback
        if info2.p[3] != 0.0 and info2.k[0] != 0.0:
            baseline = -info2.p[3] / info2.k[0]
            self.get_logger().info(f'VIO: stereo baseline = {baseline:.4f} m')

        from scipy.spatial.transform import Rotation as R
        right_translation = [baseline, 0.0, 0.0]
        right_rotation = R.identity().as_quat()  # [x,y,z,w]
        cam_right.rig_from_camera = vslam.Pose(
            rotation=right_rotation.tolist(),
            translation=right_translation)

        # IMU calibration
        imu_cal = vslam.ImuCalibration()
        imu_cal.rig_from_imu = vslam.Pose(
            rotation=[0.0, 0.0, 0.0, 1.0],
            translation=[0.0, 0.0, 0.0])
        imu_cal.gyroscope_noise_density = IMU_GYROSCOPE_NOISE_DENSITY
        imu_cal.gyroscope_random_walk = IMU_GYROSCOPE_RANDOM_WALK
        imu_cal.accelerometer_noise_density = IMU_ACCELEROMETER_NOISE_DENSITY
        imu_cal.accelerometer_random_walk = IMU_ACCELEROMETER_RANDOM_WALK
        imu_cal.frequency = IMU_FREQUENCY

        # Try to get IMU-to-camera extrinsics from TF
        try:
            imu_tf = self.tf_buffer.lookup_transform(
                'camera_infra1_optical_frame',
                'camera_gyro_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            t = imu_tf.transform.translation
            q = imu_tf.transform.rotation
            imu_cal.rig_from_imu = vslam.Pose(
                rotation=[q.x, q.y, q.z, q.w],
                translation=[t.x, t.y, t.z])
            self.get_logger().info(
                f'VIO: rig_from_imu from TF: t=[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}]')
        except TransformException:
            self.get_logger().warn(
                'VIO: Could not get IMU extrinsics from TF, using identity translation')

        # Build rig
        rig = vslam.Rig()
        rig.cameras = [cam_left, cam_right]
        rig.imus = [imu_cal]

        # Inertial odometry config
        odom_cfg = vslam.Tracker.OdometryConfig(
            async_sba=True,
            enable_final_landmarks_export=True,
            odometry_mode=vslam.Tracker.OdometryMode.Inertial,
            rectified_stereo_camera=True,
            debug_imu_mode=False
        )

        slam_cfg = None
        if self.enable_slam:
            slam_cfg = vslam.Tracker.SlamConfig(sync_mode=False)
            self.get_logger().info('SLAM enabled with async mode')

        self.tracker = vslam.Tracker(rig, odom_cfg, slam_cfg)
        self.get_logger().info('cuVSLAM Tracker initialized (VIO / Inertial mode)')

        # Cleanup info subscriptions
        self.destroy_subscription(self._infra1_info_sub)
        self.destroy_subscription(self._infra2_info_sub)

        # Start IMU subscription immediately (before TF lookup completes)
        self._start_imu_subscriber()

        # Look up base_link → camera_optical_frame
        self._lookup_camera_tf()

    def _start_imu_subscriber(self):
        """Start IMU subscriber for VIO mode."""
        imu_topic = self.get_parameter('imu_topic').value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.create_subscription(Imu, imu_topic, self._imu_callback, qos)
        self.get_logger().info(f'VIO: subscribed to IMU on {imu_topic}')

    def _imu_callback(self, msg: Imu):
        """Buffer IMU measurements for VIO mode."""
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._imu_lock:
            self._imu_buffer.append((
                timestamp_ns,
                [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            ))

    def _register_buffered_imu(self, up_to_timestamp_ns: int):
        """Register all buffered IMU measurements up to the given timestamp with the tracker."""
        with self._imu_lock:
            measurements = list(self._imu_buffer)
            self._imu_buffer.clear()

        imu_meas = vslam.ImuMeasurement()
        registered = 0
        for ts_ns, accel, gyro in measurements:
            if ts_ns <= self._last_registered_imu_ts:
                continue
            if ts_ns > up_to_timestamp_ns:
                # Put back measurements that are after the image timestamp
                with self._imu_lock:
                    remaining = [(t, a, g) for t, a, g in measurements
                                 if t > up_to_timestamp_ns]
                    for item in remaining:
                        self._imu_buffer.appendleft(item)
                break
            imu_meas.timestamp_ns = ts_ns
            imu_meas.linear_accelerations = accel
            imu_meas.angular_velocities = gyro
            try:
                self.tracker.register_imu_measurement(0, imu_meas)
                self._last_registered_imu_ts = ts_ns
                registered += 1
            except Exception as e:
                self.get_logger().warn(f'IMU register error: {e}', throttle_duration_sec=5.0)
                break
        return registered

    def _start_vio_subscribers(self):
        """Start stereo IR image subscribers for VIO mode."""
        infra1_topic = self.get_parameter('infra1_topic').value
        infra2_topic = self.get_parameter('infra2_topic').value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.infra1_sub = Subscriber(self, Image, infra1_topic, qos_profile=qos)
        self.infra2_sub = Subscriber(self, Image, infra2_topic, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.infra1_sub, self.infra2_sub],
            queue_size=10,
            slop=0.01  # 10ms — stereo should be tightly synced
        )
        self.sync.registerCallback(self._vio_image_callback)
        self.get_logger().info(
            f'VIO subscribed: {infra1_topic} + {infra2_topic}')

    # ------------------------------------------------------------------
    # Common TF lookup
    # ------------------------------------------------------------------

    def _lookup_camera_tf(self):
        """Look up the static TF from base_link to camera optical frame."""
        self._tf_lookup_attempts = 0
        self._tf_lookup_timer = self.create_timer(1.0, self._tf_lookup_tick)

    def _tf_lookup_tick(self):
        """Timer callback for TF lookup — allows spinning between attempts."""
        self._tf_lookup_attempts += 1
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            self.base_T_camera = tf_to_matrix(tf_msg)
            self.get_logger().info(
                f'Got TF {self.base_frame} -> {self.camera_optical_frame}: '
                f't=[{tf_msg.transform.translation.x:.3f}, '
                f'{tf_msg.transform.translation.y:.3f}, '
                f'{tf_msg.transform.translation.z:.3f}]')
            self._tf_lookup_timer.cancel()
            if self.use_imu:
                self._start_vio_subscribers()
            else:
                self._start_rgbd_subscribers()
            return
        except TransformException:
            self.get_logger().warn(
                f'Waiting for TF {self.base_frame} -> {self.camera_optical_frame} '
                f'(attempt {self._tf_lookup_attempts}/15)...')

        if self._tf_lookup_attempts >= 15:
            self.get_logger().warn(
                'Could not get camera TF. Using identity (camera = base_link).')
            self.base_T_camera = np.eye(4)
            self._tf_lookup_timer.cancel()
            if self.use_imu:
                self._start_vio_subscribers()
            else:
                self._start_rgbd_subscribers()

    # ------------------------------------------------------------------
    # Image callbacks (mode-specific)
    # ------------------------------------------------------------------

    def _rgbd_image_callback(self, color_msg: Image, depth_msg: Image):
        """Process synchronized color + depth images (RGBD mode)."""
        if self.tracker is None:
            return

        self.frame_count += 1
        if self.frame_count <= self.warmup_frames:
            if self.frame_count == self.warmup_frames:
                self.get_logger().info(
                    f'Warmup complete ({self.warmup_frames} frames). Tracking started.')
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        timestamp_ns = color_msg.header.stamp.sec * 1_000_000_000 + \
                        color_msg.header.stamp.nanosec

        try:
            odom_estimate, slam_pose = self.tracker.track(
                timestamp_ns,
                images=[color_img],
                depths=[depth_img]
            )
        except Exception as e:
            self.get_logger().error(f'cuVSLAM track error: {e}')
            return

        self._process_pose_estimate(
            odom_estimate, timestamp_ns, color_msg.header.stamp,
            slam_pose=slam_pose, images=[color_img])

    def _vio_image_callback(self, infra1_msg: Image, infra2_msg: Image):
        """Process synchronized stereo IR images (VIO mode)."""
        if self.tracker is None:
            return

        self.frame_count += 1
        if self.frame_count <= self.warmup_frames:
            if self.frame_count == self.warmup_frames:
                self.get_logger().info(
                    f'VIO warmup complete ({self.warmup_frames} frames). Tracking started.')
            return

        try:
            ir_left = self.bridge.imgmsg_to_cv2(infra1_msg, desired_encoding='mono8')
            ir_right = self.bridge.imgmsg_to_cv2(infra2_msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        timestamp_ns = infra1_msg.header.stamp.sec * 1_000_000_000 + \
                        infra1_msg.header.stamp.nanosec

        # Register all buffered IMU measurements up to this image timestamp
        n_imu = self._register_buffered_imu(timestamp_ns)

        try:
            odom_estimate, slam_pose = self.tracker.track(
                timestamp_ns,
                images=[ir_left, ir_right]
            )
        except Exception as e:
            self.get_logger().error(f'cuVSLAM VIO track error: {e}')
            return

        self._process_pose_estimate(
            odom_estimate, timestamp_ns, infra1_msg.header.stamp,
            slam_pose=slam_pose, images=[ir_left, ir_right])

    def _process_pose_estimate(self, odom_estimate, timestamp_ns: int, stamp,
                               slam_pose=None, images=None):
        """Common pose processing for both RGBD and VIO modes."""
        if odom_estimate.world_from_rig is None:
            if not self.tracking_lost:
                self.get_logger().warn('Tracking lost!')
                self.tracking_lost = True
            return

        if self.tracking_lost:
            self.get_logger().info('Tracking recovered!')
            self.tracking_lost = False

        # Store latest images for potential localization
        if images is not None:
            self._last_images_for_localize = images

        # Auto-localization: keep retrying every N frames until success
        if (self.auto_localize and not self._relocalized
                and not self._localization_in_progress and self.enable_slam
                and self._has_saved_map()):
            frames_since_warmup = self.frame_count - self.warmup_frames
            # First attempt immediately, then retry every 150 frames (~5s at 30fps)
            if (not self._localization_attempted
                    or (frames_since_warmup > 0
                        and frames_since_warmup % 150 == 0)):
                self._localization_attempted = True
                self._attempt_localization(images)

        # Use SLAM pose if relocalized and available (better accuracy with loop closures)
        if self._relocalized and slam_pose is not None:
            vslam_world_T_camera = pose_to_matrix(slam_pose)
        else:
            odom_pose = odom_estimate.world_from_rig.pose
            vslam_world_T_camera = pose_to_matrix(odom_pose)

        camera_T_base = np.linalg.inv(self.base_T_camera)
        cuvslam_T_base = self.base_T_camera @ vslam_world_T_camera @ camera_T_base
        map_T_base_raw = self.initial_pose @ cuvslam_T_base

        # Ground constraint: strip pitch/roll drift from cuVSLAM RGBD
        # (no IMU → no gravity reference → pitch/roll accumulate)
        map_T_base = self._ground_constrain(map_T_base_raw)

        self._publish_odom(map_T_base, timestamp_ns, stamp)

        if self.publish_tf:
            self._publish_map_to_odom_tf(map_T_base, stamp)

    def _publish_odom(self, map_T_base: np.ndarray, timestamp_ns: int,
                      stamp):
        """Publish visual odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = float(map_T_base[0, 3])
        odom_msg.pose.pose.position.y = float(map_T_base[1, 3])
        odom_msg.pose.pose.position.z = float(map_T_base[2, 3])

        # Orientation
        q = matrix_to_quat(map_T_base)
        odom_msg.pose.pose.orientation = Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

        # Velocity estimation from consecutive poses
        if self.prev_pose_matrix is not None and self.prev_timestamp_ns is not None:
            dt = (timestamp_ns - self.prev_timestamp_ns) / 1e9
            if dt > 0:
                dp = map_T_base[:3, 3] - self.prev_pose_matrix[:3, 3]
                odom_msg.twist.twist.linear = Vector3(
                    x=float(dp[0] / dt),
                    y=float(dp[1] / dt),
                    z=float(dp[2] / dt))

        self.prev_pose_matrix = map_T_base.copy()
        self.prev_timestamp_ns = timestamp_ns

        self.odom_pub.publish(odom_msg)

    @staticmethod
    def _ground_constrain(T: np.ndarray) -> np.ndarray:
        """Project 6-DOF pose to ground plane (x, y, yaw only).

        cuVSLAM RGBD without IMU drifts in pitch/roll. This keeps only the
        horizontal components so the map frame stays gravity-aligned.
        """
        fwd = T[:3, 0]
        yaw = np.arctan2(fwd[1], fwd[0])
        cy, sy = float(np.cos(yaw)), float(np.sin(yaw))
        G = np.eye(4, dtype=T.dtype)
        G[0, 0] = cy;  G[0, 1] = -sy
        G[1, 0] = sy;  G[1, 1] = cy
        G[0, 3] = T[0, 3]
        G[1, 3] = T[1, 3]
        return G

    def _publish_map_to_odom_tf(self, map_T_base: np.ndarray, stamp):
        """Compute and publish map → odom TF (replacing AMCL).

        When the base node is offline (no wheel odometry), this also publishes
        an identity odom → base_link TF so that the full TF tree stays
        connected for visualization tools like RViz/Foxglove.
        """
        odom_T_base = np.eye(4, dtype=np.float64)

        if self._odom_base_fallback:
            try:
                odom_tf = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.02))
                odom_T_base = tf_to_matrix(odom_tf)
                t = odom_T_base[:3, 3]
                if np.linalg.norm(t) > 0.001:
                    self._odom_base_fallback = False
                    self.get_logger().info(
                        'Detected real odom→base_link, exiting fallback mode')
            except TransformException:
                pass
        else:
            try:
                odom_tf = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                odom_T_base = tf_to_matrix(odom_tf)
            except TransformException:
                self._odom_base_fallback = True
                odom_T_base = np.eye(4, dtype=np.float64)
                self.get_logger().warn(
                    'Lost odom→base_link, re-entering fallback mode')

        if self._odom_base_fallback:
            odom_base_tf = TransformStamped()
            odom_base_tf.header.stamp = stamp
            odom_base_tf.header.frame_id = self.odom_frame
            odom_base_tf.child_frame_id = self.base_frame
            odom_base_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(odom_base_tf)

        map_T_odom = map_T_base @ np.linalg.inv(odom_T_base)

        q = matrix_to_quat(map_T_odom)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = float(map_T_odom[0, 3])
        tf_msg.transform.translation.y = float(map_T_odom[1, 3])
        tf_msg.transform.translation.z = float(map_T_odom[2, 3])
        tf_msg.transform.rotation = Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

        self.tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------
    # Relocalization
    # ------------------------------------------------------------------

    def _has_saved_map(self) -> bool:
        """Check if a saved cuVSLAM map exists in map_folder."""
        if not os.path.isdir(self.map_folder):
            return False
        contents = os.listdir(self.map_folder)
        return len(contents) > 0

    def _get_initial_pose_path(self) -> str:
        """Path to the saved initial_pose JSON file alongside the map."""
        return os.path.join(self.map_folder, 'initial_pose.json')

    def _save_initial_pose(self):
        """Save current initial_pose to JSON alongside the map."""
        path = self._get_initial_pose_path()
        data = {
            'initial_pose': self.initial_pose.tolist(),
            'description': 'Transform from cuVSLAM map frame to occupancy grid frame'
        }
        try:
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Initial pose saved to {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save initial_pose: {e}')

    def _load_initial_pose(self) -> bool:
        """Load initial_pose from JSON saved with the map."""
        path = self._get_initial_pose_path()
        if not os.path.exists(path):
            return False
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            self.initial_pose = np.array(data['initial_pose'], dtype=np.float64)
            x = self.initial_pose[0, 3]
            y = self.initial_pose[1, 3]
            yaw = np.arctan2(self.initial_pose[1, 0], self.initial_pose[0, 0])
            self.get_logger().info(
                f'Loaded saved initial_pose: x={x:.2f}, y={y:.2f}, yaw={np.degrees(yaw):.1f}°')
            return True
        except Exception as e:
            self.get_logger().warn(f'Failed to load initial_pose: {e}')
            return False

    def _attempt_localization(self, images: Optional[List[np.ndarray]] = None):
        """Attempt to localize in a previously saved cuVSLAM map."""
        if not self._has_saved_map():
            self.get_logger().info(
                f'No saved map in {self.map_folder}, skipping auto-localization')
            return
        if images is None:
            self.get_logger().warn('No images available for localization')
            return

        self._localization_in_progress = True
        self.get_logger().info(
            f'Attempting localization in saved map: {self.map_folder} '
            f'(radius={self.localize_radius}m)...')

        # Build guess pose from initial_x/y/yaw parameters
        ix = self.get_parameter('initial_x').value
        iy = self.get_parameter('initial_y').value
        iyaw = self.get_parameter('initial_yaw').value
        from scipy.spatial.transform import Rotation as R
        guess_rotation = R.from_euler('z', iyaw).as_quat()  # [x,y,z,w]
        guess_pose = vslam.Pose(
            rotation=guess_rotation.tolist(),
            translation=[ix, iy, 0.0])

        settings = vslam.Tracker.SlamLocalizationSettings(
            horizontal_search_radius=self.localize_radius,
            vertical_search_radius=1.0,
            horizontal_step=0.5,
            vertical_step=0.5,
            angular_step_rads=self.localize_angular_step
        )

        def on_localize_done(result_pose, error_msg):
            self._localization_in_progress = False
            if result_pose is not None:
                self._relocalized = True
                self.get_logger().info(
                    f'Relocalization SUCCESS! '
                    f'Position: [{result_pose.translation[0]:.2f}, '
                    f'{result_pose.translation[1]:.2f}, '
                    f'{result_pose.translation[2]:.2f}]')
                # Load the saved initial_pose that was used when the map was built
                self._load_initial_pose()
            else:
                self.get_logger().warn(
                    f'Relocalization failed: {error_msg}. '
                    f'Using parameter-based initial_pose or manual /initialpose.')

        try:
            self.tracker.localize_in_map(
                self.map_folder,
                guess_pose,
                images,
                settings,
                on_localize_done
            )
        except Exception as e:
            self._localization_in_progress = False
            self.get_logger().error(f'localize_in_map exception: {e}')

    def _localize_service_callback(self, request, response):
        """Service to manually trigger relocalization."""
        if self.tracker is None or not self.enable_slam:
            response.success = False
            response.message = 'SLAM not enabled or tracker not initialized'
            return response

        if not self._has_saved_map():
            response.success = False
            response.message = f'No saved map in {self.map_folder}'
            return response

        if self._localization_in_progress:
            response.success = False
            response.message = 'Localization already in progress'
            return response

        images = self._last_images_for_localize
        if images is None:
            response.success = False
            response.message = 'No images available yet (wait for tracking to start)'
            return response

        self.get_logger().info('Manual localization triggered via service...')
        self._attempt_localization(images)

        # Wait for async result (with timeout)
        for _ in range(100):  # 10 seconds max
            time.sleep(0.1)
            if not self._localization_in_progress:
                break

        if self._relocalized:
            response.success = True
            response.message = 'Relocalization successful!'
        else:
            response.success = False
            response.message = 'Relocalization failed or timed out'
        return response

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _initialpose_callback(self, msg):
        """Handle /initialpose from RViz '2D Pose Estimate' to re-align map origin."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = np.cos(yaw), np.sin(yaw)
        self.initial_pose = np.array([
            [c, -s, 0, p.x],
            [s,  c, 0, p.y],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ], dtype=np.float64)
        self.get_logger().info(
            f'Initial pose updated: x={p.x:.2f}, y={p.y:.2f}, yaw={np.degrees(yaw):.1f}°')

    def save_map_callback(self, request, response):
        """Service to save SLAM map (also saves current initial_pose)."""
        if self.tracker is None or not self.enable_slam:
            response.success = False
            response.message = 'SLAM not enabled or tracker not initialized'
            return response

        self.get_logger().info(f'Saving SLAM map to {self.map_folder}...')
        save_done = threading.Event()
        save_result = [False]

        def on_save_done(success):
            save_result[0] = success
            save_done.set()

        self.tracker.save_map(self.map_folder, on_save_done)
        save_done.wait(timeout=30.0)

        if save_result[0]:
            # Also save the initial_pose so it can be restored on relocalization
            self._save_initial_pose()

        response.success = save_result[0]
        response.message = (
            f'Map + initial_pose saved to {self.map_folder}' if save_result[0]
            else 'Map save failed')
        self.get_logger().info(response.message)
        return response

    def reset_callback(self, request, response):
        """Service to reset the tracker (re-initialize on next camera_info)."""
        self.tracker = None
        self.frame_count = 0
        self.prev_pose_matrix = None
        self.prev_timestamp_ns = None
        self.tracking_lost = False
        self._last_registered_imu_ts = 0
        with self._imu_lock:
            self._imu_buffer.clear()

        if self.use_imu:
            self._init_vio_mode()
        else:
            self._init_rgbd_mode()

        response.success = True
        response.message = 'cuVSLAM tracker reset. Waiting for camera_info...'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = CuvslamOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down cuVSLAM node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
