#!/usr/bin/env python3.10
"""
Standalone test for cuVSLAM on the wheeled-legged robot.
Tests cuVSLAM RGBD tracking using RealSense ROS2 topics WITHOUT Nav2.

Prerequisites:
  1. RealSense camera node running:
     ros2 launch realsense2_camera rs_launch.py \
       config_file:=/home/robotester1/legged_robot/realsense/realsense_params.yaml

Usage:
  python3.10 test_cuvslam_standalone.py
"""

import sys
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo

import cuvslam as vslam


class CuvslamTestNode(Node):
    def __init__(self):
        super().__init__('cuvslam_test')
        self.bridge = CvBridge()
        self.tracker: Optional[vslam.Tracker] = None
        self.frame_count = 0
        self.track_count = 0
        self.warmup = 30

        self.get_logger().info(f'cuVSLAM version: {vslam.get_version()[0]}')
        self.get_logger().info('Waiting for camera_info...')

        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.on_camera_info, 10)

    def on_camera_info(self, msg: CameraInfo):
        if self.tracker is not None:
            return

        self.get_logger().info(
            f'Camera: {msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
            f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')

        cam = vslam.Camera()
        cam.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
        cam.size = (msg.width, msg.height)
        cam.focal = (msg.k[0], msg.k[4])
        cam.principal = (msg.k[2], msg.k[5])

        rig = vslam.Rig(cameras=[cam], imus=[])

        rgbd_settings = vslam.Tracker.OdometryRGBDSettings()
        rgbd_settings.depth_scale_factor = 1000.0
        rgbd_settings.depth_camera_id = 0
        rgbd_settings.enable_depth_stereo_tracking = False

        cfg = vslam.Tracker.OdometryConfig(
            async_sba=True,
            odometry_mode=vslam.Tracker.OdometryMode.RGBD,
            rgbd_settings=rgbd_settings
        )
        self.tracker = vslam.Tracker(rig, cfg)
        self.get_logger().info('Tracker initialized! Starting image subscription...')

        self.destroy_subscription(self.cam_info_sub)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.color_sub = Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos)
        self.depth_sub = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_images)

    def on_images(self, color_msg: Image, depth_msg: Image):
        self.frame_count += 1
        if self.frame_count <= self.warmup:
            if self.frame_count % 10 == 0:
                self.get_logger().info(f'Warmup: {self.frame_count}/{self.warmup}')
            return

        color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        ts_ns = color_msg.header.stamp.sec * 1_000_000_000 + \
                color_msg.header.stamp.nanosec

        odom_est, slam_pose = self.tracker.track(
            ts_ns, images=[color], depths=[depth])

        self.track_count += 1

        if odom_est.world_from_rig is None:
            self.get_logger().warn(f'Frame {self.track_count}: TRACKING LOST')
            return

        pose = odom_est.world_from_rig.pose
        t = pose.translation
        q = pose.rotation

        if self.track_count % 30 == 0 or self.track_count <= 5:
            self.get_logger().info(
                f'Frame {self.track_count}: '
                f'pos=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}) '
                f'quat=({q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f})')


def main():
    rclpy.init()
    node = CuvslamTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Tracked {node.track_count} frames. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
