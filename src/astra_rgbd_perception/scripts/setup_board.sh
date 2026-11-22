#!/usr/bin/env bash
## @file setup_board.sh
## @brief Reproducible board setup for the Astra Pro RGB-D project.
## @details Clones the pinned third-party repositories, applies patches, builds
## libuvc, installs the Orbbec udev rule, and runs catkin_make. Optionally
## downloads YOLOv5n ONNX and builds the TensorRT engine.
##
## Run on the Xavier NX from the project repository:
##     bash src/astra_rgbd_perception/scripts/setup_board.sh
##
## Environment variables:
## - `ASTRA_RGBD_WS`: target workspace (default: repository root)
## - `SKIP_APT=1`: dependencies already installed
## - `REBUILD_LIBUVC=1`: force rebuilding libuvc
## - `BUILD_ENGINE=1`: download ONNX and build the TensorRT engine
##
## The script does not upgrade JetPack/CUDA/TensorRT and does not touch ~/hs/*.
## @copyright MIT License, 2026.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
WS="${ASTRA_RGBD_WS:-$REPO_ROOT}"
SRC="$WS/src"
PATCH_DIR="$REPO_ROOT/patches"

ASTRA_COMMIT="210f530144314da7c70072215bacba12130d9b20"
VISION_COMMIT="0818d3a1a76017c62e6a0fb2fdc2d343892b4205"

echo "Repository: $REPO_ROOT"
echo "Workspace:  $WS"

if [ ! -f "$REPO_ROOT/src/astra_rgbd_perception/package.xml" ]; then
  echo "ERROR: src/astra_rgbd_perception/package.xml not found."
  echo "Run this script from the project repository."
  exit 1
fi

# ---------------------------------------------------------------- dependencies
if [ "${SKIP_APT:-0}" != "1" ]; then
  echo "=== installing apt dependencies ==="
  if ! sudo apt-get install -y --no-install-recommends \
      libdw-dev libelf-dev libudev-dev libusb-1.0-0-dev libeigen3-dev libgflags-dev \
      v4l-utils ros-melodic-backward-ros ros-melodic-rgbd-launch \
      ros-melodic-image-geometry ros-melodic-camera-info-manager \
      ros-melodic-image-publisher; then
    cat >&2 <<'MSG'
apt failed. On this board a broken v2rayn package can block apt.
Do NOT run "apt --fix-broken install" blindly. Use the safe workaround in
docs/installation.md section 2.1:

  mkdir -p /tmp/debs && cd /tmp/debs
  apt-get download libelf-dev libdw-dev libudev-dev libv4l2rds0 v4l-utils \
    ros-melodic-backward-ros ros-melodic-rgbd-launch
  sudo dpkg -i /tmp/debs/*.deb

Then rerun with SKIP_APT=1.
MSG
    exit 1
  fi
fi

# ------------------------------------------------------------------- libraries
mkdir -p "$SRC"

clone_repo() {
  local url="$1" dir="$2" branch="$3" commit="$4"
  if [ ! -d "$dir/.git" ]; then
    echo "=== cloning $url ==="
    if [ -n "$branch" ]; then
      git clone -b "$branch" "$url" "$dir"
    else
      git clone "$url" "$dir"
    fi
  fi
  local current
  current=$(git -C "$dir" rev-parse HEAD)
  if [ "$current" != "$commit" ]; then
    echo "=== checking out $dir @ $commit ==="
    git -C "$dir" fetch --all --tags || true
    git -C "$dir" checkout "$commit"
  fi
}

clone_repo https://github.com/orbbec/ros_astra_camera.git \
  "$SRC/ros_astra_camera" "" "$ASTRA_COMMIT"
clone_repo https://github.com/ros-perception/vision_opencv.git \
  "$SRC/vision_opencv" "melodic" "$VISION_COMMIT"

apply_patch() {
  local dir="$1" patch="$2"
  if [ ! -f "$patch" ]; then
    echo "ERROR: missing patch $patch"
    return 1
  fi
  if git -C "$dir" apply --check "$patch" 2>/dev/null; then
    echo "=== applying $(basename "$patch") ==="
    git -C "$dir" apply "$patch"
  elif git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
    echo "=== $(basename "$patch") already applied ==="
  else
    echo "ERROR: cannot apply $patch (conflict). Inspect the repository." >&2
    return 1
  fi
}

apply_patch "$SRC/ros_astra_camera" "$PATCH_DIR/ros_astra_camera.patch"
apply_patch "$SRC/vision_opencv" "$PATCH_DIR/vision_opencv_opencv4.patch"

# libuvc (bundled with ros_astra_camera)
if [ "${REBUILD_LIBUVC:-0}" = "1" ] || ! pkg-config --exists libuvc; then
  echo "=== building libuvc ==="
  rm -rf /tmp/libuvc_build
  mkdir -p /tmp/libuvc_build
  unzip -q "$SRC/ros_astra_camera/dependencies/libuvc_master_d3318ae72.zip" \
    -d /tmp/libuvc_build
  cd /tmp/libuvc_build/libuvc
  rm -rf build
  mkdir build && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
           -DBUILD_EXAMPLES=OFF
  make -j4
  sudo make install
  sudo ldconfig
else
  echo "=== libuvc already installed: $(pkg-config --modversion libuvc) ==="
fi

# udev
echo "=== installing udev rule ==="
if [ -f /etc/udev/rules.d/56-orbbec-usb.rules ]; then
  sudo cp -a /etc/udev/rules.d/56-orbbec-usb.rules \
            "/etc/udev/rules.d/56-orbbec-usb.rules.bak.$(date +%Y%m%d%H%M%S)"
fi
sudo cp "$SRC/ros_astra_camera/56-orbbec-usb.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Make sure our package is inside the workspace.
if [ "$WS" != "$REPO_ROOT" ]; then
  echo "=== copying astra_rgbd_perception into $SRC ==="
  rm -rf "$SRC/astra_rgbd_perception"
  cp -a "$REPO_ROOT/src/astra_rgbd_perception" "$SRC/"
fi

# ---------------------------------------------------------------------- build
echo "=== catkin_make ==="
# shellcheck disable=SC1091
source /opt/ros/melodic/setup.bash
cd "$WS"
catkin_make -j4

# --------------------------------------------------------------------- engine
if [ "${BUILD_ENGINE:-0}" = "1" ]; then
  echo "=== building TensorRT engine (this takes 15-25 minutes) ==="
  mkdir -p "$WS/models"
  if [ ! -f "$WS/models/yolov5n.onnx" ]; then
    curl -L --retry 3 -o "$WS/models/yolov5n.onnx" \
      https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx
  fi
  export PATH=/usr/local/cuda/bin:$PATH
  /usr/src/tensorrt/bin/trtexec \
    --onnx="$WS/models/yolov5n.onnx" \
    --saveEngine="$WS/models/yolov5n_fp16.engine" \
    --fp16
fi

cat <<EOF

=== setup complete ===

Next steps:
  1. (recommended) calibrate the color camera:
     python2 $WS/src/astra_rgbd_perception/scripts/calibrate_rgb.py \\
       --cols 9 --rows 6 --square 0.025 --timeout 300
  2. start the system:
     source /opt/ros/melodic/setup.bash
     source $WS/devel/setup.bash
     roslaunch astra_rgbd_perception system.launch use_rviz:=false

Models:
  ONNX / engine live in $WS/models (not committed).
  If BUILD_ENGINE was not used:
    curl -L -o $WS/models/yolov5n.onnx \\
      https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx
    /usr/src/tensorrt/bin/trtexec --onnx=$WS/models/yolov5n.onnx \\
      --saveEngine=$WS/models/yolov5n_fp16.engine --fp16
EOF