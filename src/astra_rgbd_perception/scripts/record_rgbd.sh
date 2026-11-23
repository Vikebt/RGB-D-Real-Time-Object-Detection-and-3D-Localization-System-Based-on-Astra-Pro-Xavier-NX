#!/usr/bin/env bash
## @file record_rgbd.sh
## @brief Record RGB, depth, CameraInfo and TF into a compressed rosbag.
## @details The bag is written to `$WS/bags` (or `$BAG_DIR`) using LZ4.
## @param $1 Optional duration in seconds; 0 records until Ctrl-C.
## @copyright MIT License, 2026.
# 中文：录制 RGB/深度/CameraInfo/TF 到 LZ4 压缩 rosbag。
# 用法：record_rgbd.sh [秒数]；不传参则一直录到 Ctrl-C。
# Usage: record_rgbd.sh [duration_seconds]
set -eo pipefail

WS="${ASTRA_RGBD_WS:-$HOME/astra_rgbd_ws}"
BAG_DIR="${BAG_DIR:-$WS/bags}"
DURATION="${1:-0}"
mkdir -p "$BAG_DIR"
OUT="$BAG_DIR/rgbd_$(date +%Y%m%d_%H%M%S).bag"

source /opt/ros/melodic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS/devel/setup.bash"
fi

TOPICS=(
  /camera/color/image_raw
  /camera/depth/image_raw
  /camera/color/camera_info
  /camera/depth/camera_info
  /tf
  /tf_static
)

echo "Recording to $OUT"
if [ "$DURATION" -gt 0 ]; then
  rosbag record --lz4 -O "$OUT" --duration="${DURATION}s" "${TOPICS[@]}"
else
  rosbag record --lz4 -O "$OUT" "${TOPICS[@]}"
fi

echo "Bag written: $OUT"
rosbag info "$OUT" | head -20