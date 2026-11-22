# 安装与部署

本文记录在 **Jetson Xavier NX / JetPack 4.6.1 / ROS Melodic** 上从零部署的步骤。
目标工作空间：`~/astra_rgbd_ws`。

> 快速路径：把仓库同步到 `~/astra_rgbd_ws` 后运行
> `bash src/astra_rgbd_perception/scripts/setup_board.sh`，再按第 6 节构建 engine。
> 脚本覆盖了源码克隆、补丁、libuvc、udev 和 catkin_make；系统 apt 依赖见第 2 节。

---

## 1. 前置检查

```bash
uname -m                 # aarch64
cat /etc/nv_tegra_release
ls /opt/ros/melodic/setup.bash
ls /usr/local/cuda/bin/nvcc
ls /usr/src/tensorrt/bin/trtexec
lsusb | grep -i 2bc5     # Astra Pro RGB/Depth
```

期望：

- L4T R32.7.1，ROS Melodic，CUDA 10.2，TensorRT 8.2.1。
- `2bc5:0501`（RGB）和 `2bc5:0403`（Depth）均出现。
- 20W 6 核模式（推荐）：`sudo nvpmodel -m 8`。

本项目的审计结果见 `docs/environment_report.md`。

---

## 2. 系统依赖

```bash
sudo apt install -y \
  libdw-dev libelf-dev libudev-dev libusb-1.0-0-dev libeigen3-dev libgflags-dev \
  v4l-utils ros-melodic-backward-ros ros-melodic-rgbd-launch \
  ros-melodic-image-geometry ros-melodic-camera-info-manager ros-melodic-image-publisher
```

### 2.1 当 apt 被 `v2rayn` 阻塞时

本板实测到一个特殊状态：`v2rayn` 包为 `iU`（已解包未配置），依赖 glibc 2.39 等
Ubuntu 18.04 不可能满足的版本，导致 `apt-get install` 整体失败。

安全做法是绕过 apt 解析，单独下载并安装所需 deb：

```bash
mkdir -p /tmp/debs && cd /tmp/debs
apt-get download \
  libelf-dev libdw-dev libudev-dev libv4l2rds0 v4l-utils \
  ros-melodic-backward-ros ros-melodic-rgbd-launch

sudo dpkg -i /tmp/debs/*.deb
```

验证：

```bash
dpkg -l | grep -E "libdw-dev|libudev-dev|v4l-utils|backward-ros|rgbd-launch"
```

**不要**直接运行 `sudo apt --fix-broken install`；它会尝试配置/替换 `v2rayn`，
可能删除用户包或升级系统组件。也不要为了“修好 apt”升级 glibc。

---

## 3. 获取第三方源码并打补丁

```bash
mkdir -p ~/astra_rgbd_ws/src
cd ~/astra_rgbd_ws/src

git clone https://github.com/orbbec/ros_astra_camera.git
git -C ros_astra_camera checkout 210f530144314da7c70072215bacba12130d9b20

git clone -b melodic https://github.com/ros-perception/vision_opencv.git
git -C vision_opencv checkout 0818d3a1a76017c62e6a0fb2fdc2d343892b4205
```

把本仓库的 `patches/` 放到 `~/astra_rgbd_ws/patches/`，然后：

```bash
cd ~/astra_rgbd_ws
git -C src/ros_astra_camera apply patches/ros_astra_camera.patch
git -C src/vision_opencv apply patches/vision_opencv_opencv4.patch
```

### 补丁内容

`patches/ros_astra_camera.patch`：

1. `getDepthCameraInfo()` 在 `depth_align_` 时返回彩色内参，避免产生 NaN K。
2. 初始和周期 CameraInfo 发布都设置正确的 `frame_id`（对齐时用彩色光学坐标系）。
3. UVC 模式（Astra Pro RGB）也创建彩色 `CameraInfoManager`。否则
   `OBCameraNode` 拿不到 `rgb_camera.yaml`，depth CameraInfo 的 K 会一直是 0。

`patches/vision_opencv_opencv4.patch`：

1. `cv_bridge` 允许 OpenCV 4（JetPack 4.6 只有 OpenCV 4.1.1 开发文件）。
2. 关闭与 OpenCV 4 不兼容的 Boost.Python 绑定（项目只用 C++ cv_bridge）。

> 不要通过创建 `/usr/include/opencv` 符号链接或修改 `/opt/ros` 的方式绕过；
> 在本工作空间内重建 `vision_opencv` 是可回滚且不影响系统 ROS 的方案。

---

## 4. 编译 libuvc

`ros_astra_camera` 依赖 libuvc，但 Ubuntu 18.04 的 apt 版本可能不满足接口；使用
仓库自带源码：

```bash
rm -rf /tmp/libuvc_build && mkdir -p /tmp/libuvc_build
unzip -q ~/astra_rgbd_ws/src/ros_astra_camera/dependencies/libuvc_master_d3318ae72.zip \
  -d /tmp/libuvc_build

cd /tmp/libuvc_build/libuvc
rm -rf build           # 压缩包可能带有作者机器的旧 CMakeCache
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=OFF
make -j4
sudo make install
sudo ldconfig

pkg-config --modversion libuvc   # 0.0.6
```

---

## 5. udev 与编译

### 5.1 udev

```bash
if [ -f /etc/udev/rules.d/56-orbbec-usb.rules ]; then
  sudo cp -a /etc/udev/rules.d/56-orbbec-usb.rules \
            /etc/udev/rules.d/56-orbbec-usb.rules.bak.$(date +%Y%m%d%H%M%S)
fi
sudo cp ~/astra_rgbd_ws/src/ros_astra_camera/56-orbbec-usb.rules \
        /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 5.2 编译

```bash
cd ~/astra_rgbd_ws
source /opt/ros/melodic/setup.bash
catkin_make -j4
source devel/setup.bash
```

本项目 `CMakeLists.txt` 显式包含 `-O3`；请勿删除。没有优化时检测节点会从约
28ms/帧退化到约 110ms/帧。

### 5.3 验证

```bash
# OpenNI2 必须来自仓库自带 redist
ldd devel/lib/astra_camera/astra_camera_node | grep -i openni
# 期望：.../ros_astra_camera/include/openni2_redist/arm64/libOpenNI2.so

# cv_bridge 必须来自本工作空间
ldd devel/lib/astra_camera/astra_camera_node | grep cv_bridge
# 期望：/home/.../astra_rgbd_ws/devel/lib/libcv_bridge.so

ls devel/lib/astra_rgbd_perception/
# rgbd_sync_node detector_node depth_localizer_node
```

---

## 6. 模型与 TensorRT engine

```bash
cd ~/astra_rgbd_ws
mkdir -p models
curl -L --retry 3 \
  -o models/yolov5n.onnx \
  https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx
sha256sum models/yolov5n.onnx
# 04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262

export PATH=/usr/local/cuda/bin:$PATH
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/yolov5n.onnx \
  --saveEngine=models/yolov5n_fp16.engine \
  --fp16
```

构建时间在 Xavier NX 上约 15–25 分钟；生成 engine 约 7MB。详细 I/O 与性能见
`docs/model_deployment.md`。

验证 engine：

```bash
/usr/src/tensorrt/bin/trtexec \
  --loadEngine=models/yolov5n_fp16.engine \
  --iterations=100 --avgRuns=10
```

期望 GPU 约 7.3–7.7ms/帧，吞吐约 130 QPS（仅 engine，不含预处理/后处理）。

---

## 7. 首次运行

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash
roslaunch astra_rgbd_perception system.launch use_rviz:=false
```

新终端检查：

```bash
rostopic hz /camera/color/image_raw       # ~30Hz
rostopic hz /camera/depth/image_raw       # ~30Hz
rostopic hz /perception/detections_2d     # ~28-30Hz
rostopic echo -n1 /perception/objects_3d
```

如果彩色 `fx/fy=0`，请继续执行 `docs/user_manual.md` 第 3 节的标定。

---

## 8. 安装检查清单

- [ ] `cat /etc/nv_tegra_release` 为 R32.7.1
- [ ] ROS Melodic 可 `source`
- [ ] Astra Pro 两个 USB 设备可见
- [ ] udev 规则已安装
- [ ] libuvc pkg-config 版本 0.0.6
- [ ] `ros_astra_camera` 使用 arm64 自带 OpenNI2
- [ ] `cv_bridge` 来自工作空间，链接 OpenCV 4.1
- [ ] `catkin_make` 成功，三个节点二进制存在
- [ ] ONNX SHA256 正确
- [ ] engine 可由 `trtexec --loadEngine` 加载
- [ ] `system.launch` 启动后各 topic 有数据
- [ ] `~/.ros/camera_info/rgb_camera.yaml` 已安装
- [ ] 彩色/深度 CameraInfo 的 fx/fy 非 0（本机 fx≈568.67、fy≈568.37）

---

## 9. 已知环境问题

| 问题 | 处理 |
|---|---|
| `v2rayn` 阻塞 apt | 使用 `apt-get download` + `dpkg -i`，见 2.1 |
| libuvc 压缩包旧 CMakeCache | 删除 `build/` 后重新 cmake |
| OpenCV 3.2/4.1 cv_bridge 冲突 | 应用 `vision_opencv_opencv4.patch` 并重建 |
| OpenNI2 误用系统 2.2 | 检查 RUNPATH/ldd，重编 astra_camera |
| 彩色内参为 0 | 安装 `rgb_camera.yaml` 并重启；确认 UVC CameraInfoManager 补丁已应用 |
| depth K 为 0 而彩色正常 | 应用 `patches/ros_astra_camera.patch` 的 UVC manager 修复 |
| 深度前几帧全 0 | 等待 1 秒/丢弃冷启动帧 |
| 断电/重启后相机权限 | 确认 udev 规则仍在，`udevadm trigger` |

---

## 10. 卸载

见 `docs/user_manual.md` 第 8 节。核心是删除 `~/astra_rgbd_ws`、udev 规则、
`/usr/local/lib/libuvc*` 和标定文件；不要删除 `~/hs/*` 或 `/opt/ros`。