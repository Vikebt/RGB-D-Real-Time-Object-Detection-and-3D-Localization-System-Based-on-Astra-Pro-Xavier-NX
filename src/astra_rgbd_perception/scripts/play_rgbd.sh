#!/usr/bin/env bash
## @file play_rgbd.sh
## @brief Replay a recorded RGB-D rosbag for offline inference.
## @details Publishes `/clock` and forwards optional `rosbag play` arguments.
## @param $1 Path to the bag file.
## @param $@ Additional options for `rosbag play` (e.g. `-r 0.5`, `--loop`).
## @copyright MIT License, 2026.
# 中文：回放 rosbag，用于无相机离线推理。
# 用法：play_rgbd.sh <bag> [rosbag play 选项]，例如 -r 0.5 / --loop。
# Usage: play_rgbd.sh <bag> [rosbag play options]
set -eo pipefail

BAG="${1:?usage: play_rgbd.sh <bag> [rosbag play options...]}"
shift || true
WS="${ASTRA_RGBD_WS:-$HOME/astra_rgbd_ws}"

source /opt/ros/melodic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS/devel/setup.bash"
fi

rosbag play --clock "$BAG" "$@"