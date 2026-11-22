#!/usr/bin/env python2
# -*- coding: utf-8 -*-
## @file calibrate_rgb.py
## @brief Headless RGB chessboard calibration for the Astra Pro color stream.
## @details
## Subscribes to a `sensor_msgs/Image` topic, captures chessboard views with
## pose diversity, runs `cv2.calibrateCamera`, and writes a
## `camera_info_manager`-compatible YAML file plus an archived copy.
##
## Run on the Xavier (ROS Melodic / Python 2.7):
##
##     source /opt/ros/melodic/setup.bash
##     source ~/astra_rgbd_ws/devel/setup.bash
##     python ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/calibrate_rgb.py \
##         --cols 9 --rows 6 --square 0.025 \
##         --min-views 15 --max-views 25 --timeout 300
##
## @note The saved `view_*.png` files contain drawn corners and are intended
##       for visual inspection, not for automatic re-calibration.
## @copyright MIT License, 2026.
"""Headless RGB monocular calibration for the Astra Pro color stream.

Subscribes to a sensor_msgs/Image topic, automatically captures chessboard
views with pose diversity, runs cv2.calibrateCamera, and writes a
camera_info_manager-compatible YAML file.
"""

import argparse
import os
import sys
import time
import datetime

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Image


class RgbCalibrator(object):
    """@brief Capture chessboard views and calibrate the RGB camera.

    The calibrator subscribes to the color image topic, detects a
    `cols x rows` chessboard, accepts pose-diverse views, and runs
    `cv2.calibrateCamera` after enough views are collected.
    """

    def __init__(self, args):
        """@brief Initialize the calibrator and subscribe to the image topic.

        @param args Parsed command-line arguments (see `main`).
        """
        self.args = args
        self.pattern = (args.cols, args.rows)
        self.objp = np.zeros((args.cols * args.rows, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
        self.objpoints = []
        self.imgpoints = []
        self.signatures = []
        self.last_capture = 0.0
        self.done = False
        self.img_size = None
        self.out_dir = os.path.expanduser(args.out_dir)
        self.saved = []
        if not os.path.isdir(self.out_dir):
            os.makedirs(self.out_dir)
        self.sub = rospy.Subscriber(args.topic, Image, self.callback, queue_size=2)
        print("[calib] 已订阅 %s，请把 %dx%d 内角点棋盘格放到相机前并缓慢移动/倾斜。" %
              (args.topic, args.cols, args.rows))
        print("[calib] 目标采集 %d-%d 个视角，超时 %d 秒。" %
              (args.min_views, args.max_views, args.timeout))
        sys.stdout.flush()

    @staticmethod
    def image_to_gray(msg):
        """@brief Convert a ROS image message to an 8-bit grayscale array.

        @param msg `sensor_msgs/Image` message (`rgb8`, `bgr8` or `mono8`).
        @return Grayscale `numpy.ndarray`, or `None` for unsupported encodings.
        """
        if msg.encoding in ('rgb8', 'bgr8'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if msg.encoding in ('mono8', '8UC1'):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return None

    def board_signature(self, corners):
        """@brief Approximate the board pose for pose-diversity filtering.

        @param corners Detected chessboard corners.
        @return Tuple `(tx, ty, tilt_deg)` in metres/degrees, or `None` if
                `solvePnP` fails.
        """
        K = np.array([[600.0, 0, 320.0], [0, 600.0, 240.0], [0, 0, 1.0]])
        D = np.zeros((5, 1))
        ok, rvec, tvec = cv2.solvePnP(
            self.objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        normal = R[:, 2]
        tilt = float(np.degrees(np.arccos(min(1.0, max(-1.0, abs(normal[2]))))))
        return (float(tvec[0]), float(tvec[1]), tilt)

    def is_diverse(self, sig):
        """@brief Check whether a new board pose differs from accepted views.

        @param sig Pose signature from `board_signature`.
        @return `True` if the view is sufficiently different or unknown.
        """
        if sig is None:
            return True
        for (x, y, t) in self.signatures:
            dist = ((sig[0] - x) ** 2 + (sig[1] - y) ** 2) ** 0.5
            if dist < self.args.min_translation and abs(sig[2] - t) < self.args.min_tilt:
                return False
        return True

    def callback(self, msg):
        """@brief Process one color image and capture a view when valid.

        @param msg `sensor_msgs/Image` message.
        """
        if self.done:
            return
        now = time.time()
        if now - self.last_capture < self.args.interval:
            return
        self.last_capture = now
        gray = self.image_to_gray(msg)
        if gray is None:
            return
        self.img_size = (msg.width, msg.height)
        gray = np.ascontiguousarray(gray)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, self.pattern, flags)
        if not found:
            return
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        sig = self.board_signature(corners)
        if not self.is_diverse(sig):
            return
        idx = len(self.objpoints)
        path = os.path.join(self.out_dir, "view_%02d.png" % idx)
        vis = cv2.drawChessboardCorners(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                                        self.pattern, corners, found)
        cv2.imwrite(path, vis)
        self.objpoints.append(self.objp.copy())
        self.imgpoints.append(corners.copy())
        self.signatures.append(sig if sig is not None else (0.0, 0.0, 0.0))
        self.saved.append(path)
        print("[calib] 已采集视角 %d/%d（中心近似 %.2f,%.2f m，倾角 %.1f°）" %
              (idx + 1, self.args.max_views, self.signatures[-1][0],
               self.signatures[-1][1], self.signatures[-1][2]))
        sys.stdout.flush()
        if len(self.objpoints) >= self.args.max_views:
            self.done = True
            rospy.signal_shutdown("enough views")

    def calibrate(self):
        """@brief Run `cv2.calibrateCamera` and write the YAML files.

        @return 0 on success, 1 if there are too few views or no image size.
        """
        n = len(self.objpoints)
        print("[calib] 采集结束：%d 个有效视角" % n)
        if n < self.args.min_views:
            print("[calib] 错误：有效视角不足 %d，未生成标定文件。" % self.args.min_views)
            return 1
        if self.img_size is None:
            print("[calib] 错误：没有收到图像。")
            return 1
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.img_size, None, None,
            flags=0, criteria=criteria)
        D = D.flatten()[:5]
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        reproj = []
        for i in range(n):
            proj, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], K, D)
            err = cv2.norm(self.imgpoints[i], proj, cv2.NORM_L2) / len(proj)
            reproj.append(err)
        print("[calib] ===== 标定结果 =====")
        print("[calib] 分辨率: %dx%d" % self.img_size)
        print("[calib] RMS 重投影误差: %.4f px" % rms)
        print("[calib] 单视角误差: mean=%.4f px max=%.4f px" %
              (float(np.mean(reproj)), float(np.max(reproj))))
        print("[calib] fx=%.4f fy=%.4f cx=%.4f cy=%.4f" % (fx, fy, cx, cy))
        print("[calib] D=%s" % [round(float(v), 6) for v in D])

        Kd = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        Dd = [float(v) for v in D]
        Pd = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        data = {
            'image_width': int(self.img_size[0]),
            'image_height': int(self.img_size[1]),
            'camera_name': 'rgb_camera',
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': Kd},
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': Dd},
            'rectification_matrix': {'rows': 3, 'cols': 3,
                                     'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
            'projection_matrix': {'rows': 3, 'cols': 4, 'data': Pd},
        }
        out = os.path.expanduser(self.args.output)
        if not os.path.isdir(os.path.dirname(out)):
            os.makedirs(os.path.dirname(out))
        with open(out, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        archive = os.path.join(os.path.expanduser('~/astra_rgbd_ws/calibration'),
                               'rgb_camera_%s.yaml' % stamp)
        if not os.path.isdir(os.path.dirname(archive)):
            os.makedirs(os.path.dirname(archive))
        with open(archive, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False)
        print("[calib] 已写入: %s" % out)
        print("[calib] 已归档: %s" % archive)
        print("[calib] 请重启相机节点以加载标定文件。")
        return 0


def main():
    """@brief Parse arguments and run the calibration workflow.

    @return Process exit code from `RgbCalibrator.calibrate`.
    """
    parser = argparse.ArgumentParser(description="Headless RGB chessboard calibration")
    parser.add_argument('--cols', type=int, default=9, help='inner corners per row')
    parser.add_argument('--rows', type=int, default=6, help='inner corners per column')
    parser.add_argument('--square', type=float, default=0.025, help='square size in meters')
    parser.add_argument('--topic', default='/camera/color/image_raw')
    parser.add_argument('--min-views', type=int, default=15)
    parser.add_argument('--max-views', type=int, default=25)
    parser.add_argument('--timeout', type=int, default=300)
    parser.add_argument('--interval', type=float, default=0.5)
    parser.add_argument('--min-translation', type=float, default=0.04,
                        help='minimum board center movement (m) for a new view')
    parser.add_argument('--min-tilt', type=float, default=10.0,
                        help='minimum tilt change (deg) for a new view')
    parser.add_argument('--out-dir', default='~/astra_rgbd_ws/calibration/rgb')
    parser.add_argument('--output', default='~/.ros/camera_info/rgb_camera.yaml')
    args = parser.parse_args()

    rospy.init_node('rgb_calibrator', anonymous=True)
    calib = RgbCalibrator(args)
    rospy.Timer(rospy.Duration(args.timeout),
                lambda e: rospy.signal_shutdown('timeout'))
    rospy.spin()
    return calib.calibrate()


if __name__ == '__main__':
    sys.exit(main())