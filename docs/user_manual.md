# 用户手册

**项目**：基于 Astra Pro + Jetson Xavier NX 的 RGB-D 实时目标检测与三维定位  
**适用平台**：Jetson Xavier NX Developer Kit，JetPack 4.6.1（L4T R32.7.1），Ubuntu 18.04，ROS Melodic  
**当前版本**：0.1.0（MVP，2026-11-21）  
**工作空间**：`~/astra_rgbd_ws`

---

## 0. 先读这一节：当前能力与限制

### 已实现并实测

- Astra Pro RGB 与深度同时采集：640×480@30Hz。
- 深度硬件对齐到彩色（`depth_align:=true`），深度图为 `16UC1`、单位毫米。
- RGB-D ApproximateTime 同步，统计时间差、有效深度比例、丢帧。
- XYZ / XYZRGB 点云，约 6 万点/帧，10Hz 发布。
- YOLOv5n ONNX → TensorRT FP16，C++ 检测节点 28–30 FPS。
- 检测框中心 50% ROI 的深度中位数 + 内参反投影，发布 `Object3DArray`。
- `system.launch` 一键启动相机、同步、检测、三维定位、RViz。
- rosbag 录制与无相机离线回放。
- 性能脚本与 10 分钟稳定性采样脚本。

### 尚未完成 / 必须注意

| 项目 | 状态 | 影响 |
|---|---|---|
| 彩色相机标定 | ✅ 已完成（2026-11-22） | `~/.ros/camera_info/rgb_camera.yaml`；fx≈568.67、fy≈568.37 |
| 0.8/1.0/1.5/2.0m 实测精度 | 未执行 | 需要卷尺和实物；可用 `measure_distance.py` |
| RViz 可视化截图验收 | 部分完成 | 配置与 topic 已验证，未在显示器上截图验收 |
| 10 分钟稳定性 | ✅ 已完成 | 10 分 28 秒无崩溃/断连；双流约 30Hz，gaps=0，RSS 稳定 |

> 彩色标定已加载，`/perception/objects_3d` 的 X/Y/Z 使用真实内参与真实深度。
> 但 0.8–2.0m 卷尺实测尚未完成，最终精度验收前仍需执行。

---

## 1. 硬件与软件要求

### 硬件

- NVIDIA Jetson Xavier NX Developer Kit（16GB 模组实测可见 15GiB）。
- Orbbec Astra Pro RGB-D 相机，USB 连接。
  - RGB：`2bc5:0501`（UVC，`/dev/video0`）
  - Depth：`2bc5:0403`（OpenNI2 Vendor Specific 接口）
- 建议使用 Xavier NX 原装/足功率电源，20W 6 核模式。
- 至少 8GB 可用磁盘（TensorRT engine、rosbag）。

### 软件

| 组件 | 实测版本 |
|---|---|
| L4T / JetPack | R32.7.1 / 4.6.1 |
| Ubuntu | 18.04.6 LTS |
| ROS | Melodic（Python 2.7） |
| CUDA | 10.2.300 |
| cuDNN | 8.2.1.32 |
| TensorRT | 8.2.1（`trtexec` 在 `/usr/src/tensorrt/bin`） |
| OpenCV | 系统 4.1.1（dev）；ROS `cv_bridge` 重建后链接 4.1.1 |
| 编译器 | gcc/g++ 7.5.0，cmake 3.10.2 |

> 不要升级 JetPack、CUDA、TensorRT 或系统 Python/OpenCV 来“解决”兼容问题；
> 本项目的补丁已经处理了 OpenCV 3.2/4.1.1 与 cv_bridge 的冲突。

---

## 2. 从零安装

如果 `~/astra_rgbd_ws` 已经存在并编译完成，可跳过本节，直接看第 4 节启动。

### 2.1 创建独立工作空间

不要修改板上已有的 `~/hs/*` 工作空间：

```bash
mkdir -p ~/astra_rgbd_ws/src
cd ~/astra_rgbd_ws
```

把本项目仓库内容放到 `~/astra_rgbd_ws`（至少需要 `src/astra_rgbd_perception`、
`patches/`、`tools/`、`docs/`）。

### 2.2 安装系统依赖

正常情况：

```bash
sudo apt install -y \
  libdw-dev libelf-dev libudev-dev libusb-1.0-0-dev libeigen3-dev libgflags-dev \
  v4l-utils ros-melodic-backward-ros ros-melodic-rgbd-launch \
  ros-melodic-image-geometry ros-melodic-camera-info-manager ros-melodic-image-publisher
```

**本板曾遇到的情况**：`v2rayn` 包处于 `iU`（已解包未配置）状态，依赖 glibc 2.39，
导致所有 apt 安装被阻塞。安全处理方式是单独下载 deb 再用 `dpkg -i`，不要直接运行
`apt --fix-broken install`（它可能删除或升级系统包）：

```bash
mkdir -p /tmp/debs && cd /tmp/debs
apt-get download libelf-dev libdw-dev libudev-dev libv4l2rds0 v4l-utils \
  ros-melodic-backward-ros ros-melodic-rgbd-launch
sudo dpkg -i /tmp/debs/*.deb
```

### 2.3 获取并锁定第三方源码

```bash
cd ~/astra_rgbd_ws/src

git clone https://github.com/orbbec/ros_astra_camera.git
git -C ros_astra_camera checkout 210f530144314da7c70072215bacba12130d9b20

git clone -b melodic https://github.com/ros-perception/vision_opencv.git
git -C vision_opencv checkout 0818d3a1a76017c62e6a0fb2fdc2d343892b4205
```

应用本项目补丁（`patches/` 相对于仓库根目录）：

```bash
cd ~/astra_rgbd_ws
git -C src/ros_astra_camera apply patches/ros_astra_camera.patch
git -C src/vision_opencv apply patches/vision_opencv_opencv4.patch
```

补丁内容：

- `ros_astra_camera`：对齐深度时使用彩色内参；修复 depth `CameraInfo.frame_id`。
- `vision_opencv`：允许 cv_bridge 使用 OpenCV 4；关闭与 OpenCV 4 不兼容的
  Boost.Python 绑定（本项目只用 C++）。

### 2.4 编译 libuvc

`ros_astra_camera` 自带 libuvc 源码压缩包：

```bash
rm -rf /tmp/libuvc_build && mkdir -p /tmp/libuvc_build
unzip -q ~/astra_rgbd_ws/src/ros_astra_camera/dependencies/libuvc_master_d3318ae72.zip \
  -d /tmp/libuvc_build
cd /tmp/libuvc_build/libuvc
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=OFF
make -j4
sudo make install
sudo ldconfig
pkg-config --modversion libuvc   # 期望 0.0.6
```

> 压缩包内可能带有作者机器的旧 `build/CMakeCache.txt`，必须删除 `build/` 后重新
> cmake，否则会报 “CMakeCache.txt directory ... is different”。

### 2.5 安装 udev 规则

```bash
sudo cp ~/astra_rgbd_ws/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

如果已存在同名规则，先备份：

```bash
sudo cp -a /etc/udev/rules.d/56-orbbec-usb.rules \
          /etc/udev/rules.d/56-orbbec-usb.rules.bak.$(date +%Y%m%d%H%M%S)
```

### 2.6 编译工作空间

```bash
cd ~/astra_rgbd_ws
source /opt/ros/melodic/setup.bash
catkin_make -j4
source devel/setup.bash
```

本项目 CMake 显式加了 `-O3`。不要去掉，否则检测节点的预处理和 FP16 转换会慢到
100ms 以上。

验证：

```bash
rospack find astra_rgbd_perception
ls devel/lib/astra_rgbd_perception/
```

### 2.7 下载 ONNX 并构建 TensorRT engine

```bash
cd ~/astra_rgbd_ws
mkdir -p models
curl -L --retry 3 \
  -o models/yolov5n.onnx \
  https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx
sha256sum models/yolov5n.onnx
# 期望：04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262

export PATH=/usr/local/cuda/bin:$PATH
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/yolov5n.onnx \
  --saveEngine=models/yolov5n_fp16.engine \
  --fp16
```

- Xavier NX 上构建约 15–25 分钟，engine 约 7MB，只适用于本机 TensorRT/GPU。
- 构建日志中的关键信息：
  - `images`：`1x3x640x640`，FP16 binding
  - `output0`：`1x25200x85`，FP16 binding
- engine 的 I/O 是 FP16（`dtype=1`）；本项目的 `TrtEngine` 同时支持 FP16/FP32
  binding，会自动做主机端转换。
- 不要无条件把 `models/*.engine`、`models/*.onnx`、`bags/*.bag` 提交到 Git。

---

## 3. 彩色相机标定（本机已完成；换相机/分辨率需重做）

**本机标定结果（2026-11-22）**：

- 文件：`~/.ros/camera_info/rgb_camera.yaml`
  （归档：`~/astra_rgbd_ws/calibration/rgb_camera_20261122_134029.yaml`）
- K：`fx=568.6674, fy=568.3721, cx=316.2128, cy=261.5965`
- D：`plumb_bob [0.049442, -0.007669, 0.000592, 0.002328, -0.147309]`
- `fallback_intrinsics` 已设为 `false`；彩色与深度 CameraInfo 均使用上述内参。

以下步骤用于换相机、改分辨率或重新标定。

### 3.1 打印棋盘格

使用 `tools/chessboard_9x6_25mm.pdf`：

- A4 **横向**，打印时选择 **100% / 实际大小**，不要“适合页面”。
- 内角点 9×6，方格边长 25mm。打印后用尺子量一个方格确认 25mm。
- 如果使用自己的棋盘格，记录内角点数量和方格边长，标定时传参。

### 3.2 自动标定

先启动相机（不需要检测节点）：

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash
roslaunch astra_camera astra_pro.launch \
  depth_align:=true enable_point_cloud:=false enable_ir:=false
```

另开一个终端：

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash
python2 ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/calibrate_rgb.py \
  --cols 9 --rows 6 --square 0.025 \
  --min-views 15 --max-views 25 --timeout 300
```

在相机前缓慢移动/倾斜棋盘格，覆盖画面中心和四周，倾角 ±20–30°，距离约
0.5–1.5m。脚本每采集一个有效视角会打印进度；达到 15 个以上视角后自动标定。

输出：

- `~/.ros/camera_info/rgb_camera.yaml`（驱动会自动加载）
- `~/astra_rgbd_ws/calibration/rgb_camera_<时间>.yaml`（归档）
- 终端中的 RMS 重投影误差、fx/fy/cx/cy、畸变系数。

验收建议：RMS < 0.5px。若误差大，增加视角数量、避免运动模糊、确认方格尺寸。

### 3.3 标定后关闭 fallback（本机已完成）

本机 `config/rgbd_sync.yaml` 和 `config/localizer.yaml` 已经是
`fallback_intrinsics: false`。换相机重新标定后同样要确认。

编辑：

- `src/astra_rgbd_perception/config/rgbd_sync.yaml`
- `src/astra_rgbd_perception/config/localizer.yaml`

把：

```yaml
fallback_intrinsics: true
```

改为 `false`，然后重启系统。这样如果 CameraInfo 无效会直接报错，而不是悄悄使用
临时内参。

用以下命令确认 K 已生效：

```bash
rostopic echo -n1 /camera/color/camera_info | head -20
rostopic echo -n1 /camera/depth/camera_info | head -20
```

---

## 4. 启动系统

### 4.1 一键启动

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash
roslaunch astra_rgbd_perception system.launch
```

远程 SSH（无显示器）时关闭 RViz：

```bash
roslaunch astra_rgbd_perception system.launch use_rviz:=false
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_rviz` | `true` | 是否启动 RViz |
| `start_camera` | `true` | 是否启动 Astra 驱动 |
| `depth_align` | `true` | 深度硬件对齐到彩色 |
| `use_sync` | `true` | 是否启动 RGB-D 同步/点云 |
| `use_detector` | `true` | 是否启动 YOLO 检测 |
| `use_localizer` | `true` | 是否启动三维定位 |
| `engine_path` | `$(find astra_rgbd_perception)/../../models/yolov5n_fp16.engine` | TensorRT engine |

示例：只启动相机和 2D 检测：

```bash
roslaunch astra_rgbd_perception system.launch \
  use_sync:=false use_localizer:=false use_rviz:=false
```

### 4.2 分模块启动

```bash
# 仅相机
roslaunch astra_camera astra_pro.launch depth_align:=true

# RGB-D 同步 + 点云
roslaunch astra_rgbd_perception rgbd.launch start_camera:=false

# 仅检测（需要相机已运行）
roslaunch astra_rgbd_perception detector.launch

# 仅三维定位（需要相机 + 检测）
roslaunch astra_rgbd_perception localizer.launch
```

### 4.3 查看结果

```bash
rostopic list | grep -E '^/camera|^/perception'
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_raw
rostopic hz /perception/detections_2d
rostopic hz /perception/objects_3d
rostopic echo -n1 /perception/objects_3d
rqt_image_view /perception/image_annotated
```

`Object3DArray` 中每个目标包含：类别、置信度、2D 框、`depth_m`、有效深度像素数、
有效率、`position`（X/Y/Z，米）、`depth_valid`。`depth_valid=false` 时不要使用
其 `position`。

### 4.4 RViz

```bash
rosrun rviz rviz -d \
  ~/astra_rgbd_ws/src/astra_rgbd_perception/config/rgbd_view.rviz
```

固定坐标系：`camera_color_optical_frame`。默认显示：

- Grid、TF
- Color（原始 RGB）
- Depth（原始深度）
- Detections（`/perception/image_annotated`）
- Objects3D（`/perception/markers`）
- XYZRGB 点云（`/perception/points_xyzrgb`）
- XYZ 点云（`/perception/points_xyz`，默认关闭）

如果 RViz 中看不到点云，先确认 `rostopic hz /perception/points_xyzrgb` 有输出，
并检查 Fixed Frame 是否为 `camera_color_optical_frame`。

---

## 5. 录制与离线回放

### 5.1 录制

```bash
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/record_rgbd.sh 10
# 不带参数则一直录制，Ctrl-C 结束
```

默认保存到 `~/astra_rgbd_ws/bags/rgbd_<时间>.bag`，包含：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/camera_info`
- `/tf`、`/tf_static`

使用 LZ4 压缩；10 秒约 236MB（场景相关）。

### 5.2 离线回放

终端 1（不启动相机）：

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash
roslaunch astra_rgbd_perception system.launch \
  start_camera:=false use_rviz:=false
```

终端 2：

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/play_rgbd.sh \
  ~/astra_rgbd_ws/bags/rgbd_20261121_195513.bag -r 1
```

回放期间同样会有 `/perception/detections_2d` 和 `/perception/objects_3d`。
`-r 0.5` 可慢放，`--loop` 可循环。

---

## 6. 性能测试

```bash
# 5-10 秒采样，输出 topic 频率、同步统计、检测耗时、CPU/GPU/内存
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/benchmark.sh 10

# 10 分钟稳定性：每 30 秒记录 RSS、频率、有效率、丢帧、节点列表
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/stability_test.sh 600 30
```

结果写入 `~/astra_rgbd_ws/logs/`。实测数据见 `docs/performance_report.md`。

---

## 7. 常见问题

### 7.1 编译时报 `cv_bridge specifies '/usr/include/opencv' is not found`

说明系统 `cv_bridge` 是为 OpenCV 3.2 编译的，而 JetPack 只有 OpenCV 4.1.1。
按 2.3 节应用 `patches/vision_opencv_opencv4.patch` 并在本工作空间内重新编译
`vision_opencv`。不要用软链接 `/usr/include/opencv` 的临时方案。

### 7.2 TensorRT engine 能加载但输出全 0 / 数值异常

先看启动日志：

```bash
grep "binding .*dtype" ~/.ros/log/latest/detector-*.log
```

- `dtype=1` 表示 FP16 binding；`dtype=0` 表示 FP32。
- 本项目 `TrtEngine` 会自动处理 FP16/FP32，但 engine 必须由本机 TensorRT 构建。
- 如果自己重新导出/重建 engine，确认输入是 `1x3x640x640`、输出是
  `1x25200x85`；不匹配时修改 `config/detector.yaml` 或使用对应 engine。

### 7.3 彩色 `CameraInfo` 的 fx/fy 为 0

说明标定文件没有被加载。检查：

- `~/.ros/camera_info/rgb_camera.yaml` 是否存在，`image_width/image_height`
  是否为 640×480；
- 标定后是否重启了相机节点；
- `config/rgbd_sync.yaml` / `config/localizer.yaml` 的
  `fallback_intrinsics` 是否为 `false`。

本机正常值：`fx≈568.67, fy≈568.37, cx≈316.21, cy≈261.60`，深度 CameraInfo 的 K
应与彩色一致。若仍为 0，应用 `patches/ros_astra_camera.patch`（UVC 模式也要创建
彩色 CameraInfoManager）。

### 7.4 深度图前几帧全 0

OpenNI 深度流是冷启动，前若干帧可能全 0。连续订阅约 1 秒后即可稳定。点云/定位
节点会过滤 0 深度，不会把 0 当作有效距离。

### 7.5 `get_camera_params` 返回 NaN

Astra Pro 的这块固件没有通过 OpenNI 暴露工厂标定参数。这是本项目需要自行标定
彩色相机的原因；不要伪造内参。

### 7.6 depth `CameraInfo.frame_id` 为空或 K 为 NaN

应用 `patches/ros_astra_camera.patch`。对齐模式下深度图在彩色光学坐标系，K 应使用
彩色内参；本项目已修复。

### 7.7 apt 安装被 `v2rayn` 阻塞

见 2.2 节。不要直接 `apt --fix-broken install`，先确认它不会删除你的依赖。

### 7.8 检测帧率低

- 确认编译参数含 `-O3`：
  ```bash
  grep CXX_FLAGS ~/astra_rgbd_ws/build/astra_rgbd_perception/CMakeFiles/detector_node.dir/flags.make
  ```
- 确认 `config/detector.yaml` 中 `debug_output: false`。
- 用 `tegrastats` 查看 GR3D_FREQ/CPU 占用；确认 20W 模式。
- engine-only 基准：
  ```bash
  /usr/src/tensorrt/bin/trtexec --loadEngine=models/yolov5n_fp16.engine \
    --iterations=100 --avgRuns=10
  ```
  实测 GPU 约 7.36ms/帧。

### 7.9 OpenNI2 加载了系统 2.2 版而不是 Orbbec 自带版本

```bash
ldd ~/astra_rgbd_ws/devel/lib/astra_camera/astra_camera_node | grep -i openni
readelf -d ~/astra_rgbd_ws/devel/lib/astra_camera/astra_camera_node | grep RUNPATH
```

应指向 `src/ros_astra_camera/include/openni2_redist/arm64/libOpenNI2.so`。若指向
`/usr/lib/libOpenNI2.so`，重新编译 `ros_astra_camera`，不要手动覆盖系统库。

### 7.10 USB 带宽/断连

两个设备都在 USB 2.0 480Mbps hub 上。首版固定 640×480@30；不要同时开
1280×720 RGB + 深度 + 点云。若出现断连，降低分辨率/帧率或使用带供电的 USB hub。

---

## 8. 恢复与卸载

本项目只新增：

- `~/astra_rgbd_ws`（工作空间）
- `/etc/udev/rules.d/56-orbbec-usb.rules`
- `/usr/local/lib/libuvc.so*`、`/usr/local/include/libuvc`
- `~/.ros/camera_info/rgb_camera.yaml`（标定后）

卸载：

```bash
sudo rm -f /etc/udev/rules.d/56-orbbec-usb.rules
sudo udevadm control --reload-rules
sudo rm -f /usr/local/lib/libuvc.so* /usr/local/lib/libuvc.a
sudo rm -rf /usr/local/include/libuvc /usr/local/lib/pkgconfig/libuvc.pc
sudo ldconfig
rm -rf ~/astra_rgbd_ws
```

不要删除 `~/hs/*` 下用户已有的 ROS 工作空间，也不要覆盖 `/opt/ros/melodic`。

---

## 9. 验收状态速查

| 任务书验收项 | 状态 | 证据/说明 |
|---|---|---|
| 相机 RGB/Depth 正常 | ✅ | 30/29.8Hz，rgb8/16UC1，见 camera_configuration.md |
| 彩色相机标定 | ✅ | fx≈568.67、fy≈568.37，`~/.ros/camera_info/rgb_camera.yaml` |
| CameraInfo 正确 | ✅ | 彩色/深度对齐 K 一致；depth K 已同步 |
| 点云 XYZ/XYZRGB | ✅ | ~6 万点，10Hz，RViz 配置已提供 |
| YOLO 检测 | ✅ | 28–30 FPS，标注图见 docs/images |
| 三维定位 | ✅（精度实测待做） | 27.2 FPS，3D 成功率 82.6%，使用标定内参 |
| RViz 可视化 | ⚠️ | 配置/topic 已验证，截图待有显示器时补 |
| 录制/离线回放 | ✅ | 10s bag，无相机回放正常 |
| 0.8–2.0m 精度 | ❌ 待用户配合 | 使用 `measure_distance.py`，需要卷尺/实物 |
| 10 分钟稳定性 | ✅ | 10 分 28 秒无崩溃/断连；RSS 稳定；见 performance_report.md |
| 完整文档 | ✅ | 本手册 + docs/ 其余文档 |

最终验收前，务必完成 0.8–2.0m 实测和 RViz 现场截图。距离实测命令：

```bash
python2 ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/measure_distance.py \
  --class bottle --duration 5 --show-all
```

把物体放在卷尺量定的距离（从镜头前沿到物体正面），保持不动，脚本会输出
`median_z`、`mean_x/y` 和检测框；与卷尺值对比即可。