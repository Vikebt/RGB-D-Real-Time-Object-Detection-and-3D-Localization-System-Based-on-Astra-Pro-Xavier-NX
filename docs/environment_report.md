# 环境审计报告

- 审计日期：2026-11-21
- 审计对象：`ssh xavier`（`hs-3@192.168.20.144`）
- 审计方式：只读检查，未安装、未升级、未修改任何系统配置
- 结论：**当前环境满足任务书技术路线的基本条件，可以进入第二阶段**。主要缺口是 Orbbec 相机驱动（OpenNI2 Orbbec 插件 + `ros_astra_camera`）和少量编译依赖。

---

## 1. 硬件型号

| 项目 | 实测值 |
|---|---|
| 设备 | NVIDIA Jetson Xavier NX Developer Kit |
| SoC | `t186ref`（Xavier NX） |
| CPU | 6 核 `aarch64` |
| 内存 | 15 GiB 可见（16 GB 级模组），审计时已用 1.6 GiB，可用 13 GiB |
| 交换 | 7.7 GiB |
| 系统盘 | NVMe `/dev/nvme0n1p1`，103 GB，已用 62 GB，**可用 36 GB** |
| eMMC | `/dev/mmcblk0p1` 14 GB，可用 2.2 GB（当前未作为系统盘使用） |
| 电源模式 | `MODE_20W_6CORE`（Type 8），CPU governor `schedutil` |
| 温度传感器 | CPU/GPU/AUX/AO/PMIC/thermal-fan 均存在，`tegrastats`、`jetson_clocks`、`nvpmodel` 可用 |

## 2. JetPack / L4T / CUDA / TensorRT

| 项目 | 实测值 | 说明 |
|---|---|---|
| L4T | R32.7.1（GCID 29818004，2022-02-19） | `/etc/nv_tegra_release` |
| JetPack | 4.6.1 | `jetson_release` |
| Ubuntu | 18.04.6 LTS (bionic) | `/etc/os-release` |
| Kernel | `4.9.253-tegra` | |
| CUDA | 10.2.300 | `nvcc` 位于 `/usr/local/cuda/bin/nvcc`，非登录 shell 的 PATH 未包含 `/usr/local/cuda/bin` |
| cuDNN | 8.2.1.32 | 头文件 `/usr/include/cudnn_version.h` |
| TensorRT | 8.2.1-1+cuda10.2 | `libnvinfer*` / `libnvonnxparsers*` 已安装，`trtexec` 位于 `/usr/src/tensorrt/bin/trtexec` |
| TensorRT Python | 8.2.1.9 | `python3 -c "import tensorrt"` 成功 |
| Vulkan | 1.2.70 | |

> 注意：`jetson_release` 显示 `TensorRT: NOT_INSTALLED`，与实际不符。已通过 TensorRT 库、头文件、`trtexec` 和 Python 绑定四项交叉确认 TensorRT 8.2.1 已正确安装，该显示为 `jetson-stats` 检测缺陷，不影响使用。

## 3. ROS

| 项目 | 实测值 |
|---|---|
| 发行版 | ROS Melodic |
| 安装路径 | `/opt/ros/melodic` |
| ROS Python | **Python 2.7.17**（Melodic 原生） |
| `roscore` / `roslaunch` / `catkin_make` | 均存在且可执行 |

已安装且与本项目相关的包：

- `ros-melodic-cv-bridge`
- `ros-melodic-image-transport`
- `ros-melodic-image-pipeline`
- `ros-melodic-depth-image-proc`
- `ros-melodic-pcl-ros`
- `ros-melodic-rviz`
- `ros-melodic-message-filters`
- `ros-melodic-tf2-ros`
- `ros-melodic-rqt-image-view`
- `ros-melodic-image-geometry`
- `ros-melodic-camera-info-manager`
- `ros-melodic-image-publisher`
- `ros-melodic-dynamic-reconfigure`
- `ros-melodic-message-generation`

未安装但任务不需要：`ros-melodic-usb-cam`（本方案 RGB 由 `astra_camera` 内置的 UVC 模块采集）。

板上已有 7 个 catkin 工作空间（`~/hs/` 下：`cam_ws`、`fastlio_ws`、`gps_ws`、`livox_driver`、`lio_ws`、`mapping_ws`、`cylinder_workspace`）。**本项目不修改、不复用这些工作空间**，将新建独立的 `~/astra_rgbd_ws`。

## 4. Python / OpenCV

| 项目 | 实测值 | 备注 |
|---|---|---|
| Python 3（系统） | 3.6.9 | |
| Python 2（ROS） | 2.7.17 | Melodic 的 `rospy`、`cv_bridge`、`tf2_ros` 在此环境 |
| NumPy | 1.13.3 (py3) | 不建议升级 |
| SciPy | 0.19.1 (py3) | |
| Pillow | 5.1.0 (py3) | |
| OpenCV（系统 dev） | **4.1.1**（JetPack 自带，`pkg-config --modversion opencv4`） | 编译时**未启用 CUDA** |
| OpenCV（ROS cv_bridge 链接） | **3.2.0** | `ldd /opt/ros/melodic/lib/libcv_bridge.so` 显示 `libopencv_*.so.3.2` |
| OpenCV（Python 3） | 3.2.0（`python3-opencv`） | 与 dev 版 4.1.1 不同 |
| OpenCV 3.2 开发头文件 | **不存在** | 只有 3.2 运行库，没有 `/usr/include/opencv2` |

缺少的 Python 包：`onnx`、`onnxruntime`、`torch`、`torchvision`、`pycuda`、`openni`。  
**不在 Xavier 上安装 PyTorch / Ultralytics**；YOLO → ONNX 在 PC 上完成，Xavier 只负责 ONNX → TensorRT Engine 和 C++ 推理。

## 5. Astra Pro USB 设备信息

`lsusb`：

```
Bus 001 Device 006: ID 2bc5:0501  Astra Pro HD Camera
Bus 001 Device 007: ID 2bc5:0403  ASTRA Pro (ORBBEC)
```

| 设备 | VID:PID | 产品名 | 驱动 | 速率 | 用途 |
|---|---|---|---|---|---|
| RGB | `2bc5:0501` | Astra Pro HD Camera | `uvcvideo` | 480 Mbps（USB 2.0） | `/dev/video0`，UVC 彩色图 |
| Depth | `2bc5:0403` | ASTRA Pro | 无内核驱动（Vendor Specific `ff/00/00`） | 480 Mbps（USB 2.0） | 需 Orbbec OpenNI2 插件 |

RGB UVC 描述符支持的分辨率：1280×720、640×480、352×288、320×240、176×144、160×120；格式索引 1 为 YUYV（GUID `59555932`），另有第二种格式（索引 2，`astra_pro.launch` 默认使用 MJPEG）。

**RGB 快速采集验证（仅验证设备可用性）**：

- OpenCV `VideoCapture(0)` 打开成功
- 640×480、上报 30 FPS
- 连续读取 10 帧全部有效，格式 `uint8`、`(480, 640, 3)`
- 10 帧耗时 0.67 s（含启动开销，均值约 15 FPS，**不是正式性能结论**，第二/七阶段将规范测量）

深度设备当前状态：

- 接口 0：Vendor Specific（无内核驱动），即 OpenNI 深度数据通道
- 接口 1/2：`snd-usb-audio`（设备自带麦克风）
- 用户 `hs-3` 已在 `video` 组，`/dev/video0` 权限正常

### OpenNI2 现状

| 项目 | 实测值 |
|---|---|
| 系统 OpenNI2 | `libopenni2-0` / `libopenni2-dev` 2.2.0.33 |
| 驱动目录 | `/usr/lib/OpenNI2/Drivers` |
| 现有驱动 | `libDummyDevice.so`、`libOniFile.so`、`libPS1080.so`、`libPSLink.so`（Primesense） |
| Orbbec 驱动 | **缺失**（无 `liborbbec.so`） |
| Python `openni` 模块 | 缺失 |
| Orbbec udev 规则 | **缺失**（`/etc/udev/rules.d` 中无 Orbbec/Astra 规则） |
| `ros_astra_camera` | **未安装**（apt 源中也没有） |

结论：深度相机当前不可用，必须进入第二阶段部署 Orbbec 驱动。

## 6. 驱动源码选型（阶段 2 依据）

已从 GitHub 拉取并检查：

| 仓库 | 分支/提交 | 结论 |
|---|---|---|
| `orbbec/ros_astra_camera` | `main` @ `210f530144314da7c70072215bacba12130d9b20`（2025-06-05，版本 1.2.7） | **采用**。支持 ROS Kinetic/Melodic/Noetic，仓库内自带 **arm64 OpenNI2 运行库**：`include/openni2_redist/arm64/libOpenNI2.so`、`OpenNI2/Drivers/liborbbec.so`、`libOniFile.so`，且 `liborbbec.so` 中包含 `Orbbec Astra Pro` 设备字符串 |
| `orbbec/OpenNI_SDK` | `main` @ `a640cb3724f323dc83d410d0550b5d504894db8a`（v2.3.0.86-beat6） | 仓库仅有文档，无二进制库，**不需要单独部署** |

关键事实：

- `launch/astra_pro.launch` 正是本项目硬件组合：深度走 OpenNI2，RGB 走 UVC（`uvc_vendor_id=0x2bc5`、`uvc_product_id=0x0501`、默认 `mjpeg`）。
- CMake 将 `ORBBEC_OPENNI2_REDIST` 写入 `CMAKE_BUILD_RPATH`，编译产物应优先加载仓库自带的 `libOpenNI2.so`，并自动找到同目录下的 `liborbbec.so`。**编译后必须用 `readelf -d` / `ldd` 验证没有误用系统 2.2.0 版 OpenNI2。**
- 编译依赖：`libuvc`（仓库带 `dependencies/libuvc_master_d3318ae72.zip`）、`backward_ros`、`libdw-dev`、`libudev-dev`、ROS 的 `image_geometry` / `camera_info_manager` / `image_transport` / `rgbd_launch` 等。

## 7. 网络与权限

| 项目 | 实测值 |
|---|---|
| 代理 | `/etc/environment` 中已配置 `http://127.0.0.1:7890`（http/https/all） |
| GitHub | `https://github.com` HTTP 200 |
| raw.githubusercontent.com | 可访问 |
| PyPI | 可访问 |
| apt 源 | 清华 `ubuntu-ports` / `ros` 镜像 + NVIDIA Jetson r32.7 源，`apt-get update` 成功 |
| sudo | `hs-3` NOPASSWD 可用 |
| 磁盘余量 | 36 GB（够编译、TensorRT engine 与 rosbag，需持续关注） |

代理仅在当前用户/环境配置中生效；非交互 SSH 会读取 `/etc/environment`，已确认本轮审计中 `git clone`/`curl` 正常。

## 8. 当前缺失依赖清单

阶段 2 需要补齐：

| 依赖 | 当前状态 | 处理方式 |
|---|---|---|
| `libdw-dev` | 缺失 | apt 安装 |
| `libudev-dev` | 缺失 | apt 安装 |
| `ros-melodic-backward-ros` | 缺失 | apt 安装 |
| `ros-melodic-rgbd-launch` | 缺失 | apt 安装 |
| `libuvc` | 缺失（无 pkg-config） | 使用仓库自带 `libuvc_master_d3318ae72.zip` 源码编译，安装到 `/usr/local` |
| `v4l-utils`（`v4l2-ctl`） | 缺失 | apt 安装，用于相机格式/帧率核验 |
| Orbbec OpenNI2 + `liborbbec.so` | 缺失 | 使用 `ros_astra_camera` 自带 arm64 redist |
| `ros_astra_camera` | 缺失 | 源码编译到独立工作空间 |
| `onnx` / `torch` | 缺失 | **不在板上安装**，PC 端导出 ONNX |

## 9. 兼容性风险与对策

| 风险 | 实测依据 | 对策 |
|---|---|---|
| **OpenCV ABI 冲突** | `cv_bridge` 链接 OpenCV 3.2，而编译期 `find_package(OpenCV)` 会找到 4.1.1，且系统没有 3.2 开发头文件 | 先用现有环境编译驱动并做运行时验证。若出现 `cv_bridge` 相关崩溃/符号错误，在独立工作空间内源码编译 `vision_opencv`（melodic 分支，`cv_bridge` + `image_geometry`）并链接 OpenCV 4.1.1；**不改动 `/opt/ros` 系统安装** |
| ROS Python 版本 | Melodic 的 `rospy` 只支持 Python 2.7；Python 3 下 `tf2_ros` C 扩展导入失败 | 正式节点全部用 C++；仅独立算法/性能脚本使用 Python 3（不 import rospy） |
| TensorRT 8.2 ONNX 算子集 | TRT 8.2 + CUDA 10.2 | YOLOv5 导出时固定 opset 12、静态 640×640、batch=1、FP16；先用 `trtexec` 验证 |
| 板上无 PyTorch | 未安装 | ONNX 在 PC 导出；如需板上验证只使用 `trtexec` 与 TensorRT Python/C++ |
| USB 2.0 带宽 | 两个设备均在 480 Mbps hub | 首版 RGB/Depth 均 640×480@30；RGB 使用 MJPEG；避免同时开 1280×720 RGB + 深度 + 点云 |
| 深度单位/编码 | 尚未实测 | 第二阶段必须确认 encoding（预期 `16UC1`）、单位（mm）、`depth_scale`，禁止毫米/米混用 |
| 深度对齐 | 尚未实测 | 优先驱动 `depth_align=true` 硬件配准；否则用 `depth_image_proc`；验收标准是 RGB 框像素能正确取深度 |
| 系统 OpenNI2 误用 | 系统存在 2.2.0 版 OpenNI2 且无 Orbbec 驱动 | 编译后用 `readelf -d` 确认 RPATH 指向仓库 redist；运行时用 `ldd` 确认加载的是仓库自带 `libOpenNI2.so` |
| 磁盘/长时间运行 | 36 GB 可用 | 控制 rosbag 大小；第九阶段做 10–30 分钟稳定性测试 |
| 供电/散热 | 20W 6 核模式 | 性能测试时记录温度/降频情况，不伪造 FPS |

## 10. 阶段 2 实施计划（基于以上审计）

1. 新建独立工作空间 `~/astra_rgbd_ws/src`，不触碰 `~/hs/*`。
2. `apt install` 补 `libdw-dev libudev-dev ros-melodic-backward-ros ros-melodic-rgbd-launch v4l-utils`（禁止 `upgrade`）。
3. 用仓库自带 zip 编译 `libuvc` → `/usr/local`，`ldconfig`，记录安装内容。
4. 克隆 `ros_astra_camera` 并固定到 `210f530144314da7c70072215bacba12130d9b20`。
5. 备份并安装 `56-orbbec-usb.rules`，`udevadm reload/trigger`。
6. `catkin_make -j4`；验证二进制 RPATH 与 `ldd` 加载的 OpenNI2 来源。
7. 启动 `astra_pro.launch`（先关闭点云降负载），核验：
   - RGB、Depth、IR、color/depth `CameraInfo`、TF topic 名称
   - 各 topic 频率、分辨率、编码
   - 深度单位与内参
   - RGB 与深度时间戳差、是否对齐
   - `rqt_image_view`/`rviz` 能看到图像
   - 连续运行无 USB 断连
8. 根据实测结果决定深度对齐方案，产出 `docs/camera_configuration.md`。

**阶段 2 验收门槛**：RGB 与 Depth 同时稳定输出，`CameraInfo` 内参有效，深度单位明确，RGB 检测框像素与深度图对应关系可验证。

---

## 附：审计命令摘要

```bash
uname -a; uname -m; cat /etc/os-release; cat /etc/nv_tegra_release
source /opt/ros/melodic/setup.bash; rosversion -d
nvcc --version; cat /usr/local/cuda/version.txt
dpkg -l | grep -E 'nvinfer|tensorrt'
pkg-config --modversion opencv4
python3 --version; python2.7 --version
lsusb; lsusb -t; ls -l /dev/video*
ls -la /usr/lib/OpenNI2/Drivers
ls -l /etc/udev/rules.d/
free -h; df -h
journalctl -k  # 需要时检查 USB 内核日志
```