# Astra Pro 相机配置与采集验收

- 测试日期：2026-11-21
- 设备：Orbbec Astra Pro（序列号 `17121920428`）
- 主机：Jetson Xavier NX，JetPack 4.6.1 / L4T R32.7.1
- 驱动：`orbbec/ros_astra_camera` 1.2.7，
  commit `210f530144314da7c70072215bacba12130d9b20`
- OpenNI2：使用驱动仓库自带 arm64 redist，而非系统 `libopenni2-0` 2.2.0

---

## 1. USB 设备

```
Bus 001 Device 006: ID 2bc5:0501  Astra Pro HD Camera   (UVC RGB)
Bus 001 Device 007: ID 2bc5:0403  ASTRA Pro             (OpenNI2 Depth)
```

| 项目 | RGB | Depth |
|---|---|---|
| VID:PID | `2bc5:0501` | `2bc5:0403` |
| 产品名 | Astra Pro HD Camera | ASTRA Pro |
| 内核驱动 | `uvcvideo` | 无（Vendor Specific `ff/00/00`） |
| USB 速率 | 480 Mbps（USB 2.0） | 480 Mbps（USB 2.0） |
| 设备节点 | `/dev/video0` | 通过 OpenNI2 访问 |
| 补充接口 | — | 2 个 `snd-usb-audio`（麦克风） |

udev 规则：`/etc/udev/rules.d/56-orbbec-usb.rules`，设备权限实测
`crw-rw-rw-`。用户 `hs-3` 在 `video` 组。

---

## 2. 实际发布的 Topic

启动参数：

```bash
roslaunch astra_camera astra_pro.launch \
  depth_align:=true enable_point_cloud:=false enable_ir:=false \
  color_width:=640 color_height:=480 color_fps:=30 \
  depth_width:=640 depth_height:=480 depth_fps:=30
```

实测 topic：

| Topic | 编码/类型 | 分辨率 | 频率 |
|---|---|---|---|
| `/camera/color/image_raw` | `rgb8` | 640×480 | 30.1 Hz |
| `/camera/depth/image_raw` | `16UC1` | 640×480 | 29.7–29.8 Hz |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 640×480 | 30 Hz |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 640×480 | 30 Hz |
| `/camera/ir/camera_info` | `sensor_msgs/CameraInfo` | — | IR 关闭时仍发布 |

图像字段：

```
color: encoding=rgb8, width=640, height=480, step=1920
depth: encoding=16UC1, width=640, height=480, step=1280
```

`step=1280` 对应每像素 2 字节，确认深度为 16 位。

---

## 3. 深度单位

- 驱动日志：`format PIXEL_FORMAT_DEPTH_1_MM`。
- 实测深度值范围例如 `0..9576`，中位数视场景为数百到数千。
- 因此 `16UC1` 的原始值单位为 **毫米**。
- 本项目统一使用 `depth_scale_to_m: 0.001`：

```
Z(m) = raw_depth * 0.001
```

注意：驱动的 `depth_scale` 参数默认为 1，不能直接把它当作“米”。

---

## 4. 深度对齐

启动参数 `depth_align:=true` 时：

- 深度流通过 OpenNI 硬件配准到彩色相机。
- `/camera/depth/image_raw` 的 `frame_id` 为 `camera_color_optical_frame`。
- 深度图与彩色图像同分辨率 640×480，可按相同像素坐标取深度。
- `/camera/depth/camera_info` 的 `frame_id` 也应为
  `camera_color_optical_frame`，K 应为彩色内参。

未对齐时：

- 深度 `frame_id` 为 `camera_depth_optical_frame`。
- 本项目 `require_aligned: true`，检测框像素不能直接用于深度索引。

### 驱动修复

原始 `ros_astra_camera` 存在两个问题，已通过
`patches/ros_astra_camera.patch` 修复：

1. 周期发布 CameraInfo 时没有设置 `frame_id`，导致深度 CameraInfo 为空。
2. 对齐深度时直接使用设备返回的 NaN 彩色参数，导致 K 为 NaN。

修复后实测：

```
depth camera_info: frame_id=camera_color_optical_frame
depth K = [0.0, 0.0, 319.5, 0.0, 0.0, 239.5, 0.0, 0.0, 1.0]  # 未标定
NaN = False
```

---

## 5. 彩色相机内参（已标定）

2026-11-22 使用 `tools/chessboard_9x6_25mm.pdf`（9×6 内角点、25mm 方格）
完成标定。

- 标定文件：`~/.ros/camera_info/rgb_camera.yaml`
- 归档：`~/astra_rgbd_ws/calibration/rgb_camera_20261122_134029.yaml`
- 实测 `/camera/color/camera_info`：

```
frame_id: camera_color_optical_frame
width: 640, height: 480
distortion_model: plumb_bob
K: [568.6674, 0, 316.2128, 0, 568.3721, 261.5965, 0, 0, 1]
D: [0.049442, -0.007669, 0.000592, 0.002328, -0.147309]
P: [568.6674, 0, 316.2128, 0, 0, 568.3721, 261.5965, 0, 0, 0, 1, 0]
```

- 驱动启动日志不再出现 `rgb_camera.yaml not found`。
- `get_camera_params` 对该固件仍返回 NaN（工厂参数不可用），因此标定文件是
  真实内参来源，不能依赖设备工厂参数。
- 标定 RMS 未在归档中保存：标定脚本保存的是绘制角点后的视图，无法从这些
  标注图复算；以标定时终端输出为准。

### 5.1 UVC 模式的 CameraInfoManager

Astra Pro 的 RGB 走 UVC。原驱动只在非 UVC 模式创建彩色
`CameraInfoManager`，导致 `OBCameraNode` 内部的 `getColorCameraInfo()` 拿不到
标定，depth CameraInfo 一直为 0。已在 `patches/ros_astra_camera.patch` 中修复：
UVC 模式也创建 `rgb_camera` 的 manager，使 `depth_align` 下的深度 CameraInfo
使用真实彩色 K。

### 5.2 fallback 参数

`fallback_intrinsics` 现在为 `false`。fallback 仅用于故障兜底，不是验收数据。

---

## 6. 深度 CameraInfo 与 TF

对齐模式下实测：

| 项目 | 值 |
|---|---|
| depth image frame_id | `camera_color_optical_frame` |
| depth CameraInfo frame_id | `camera_color_optical_frame` |
| depth CameraInfo K | 等于彩色 K（fx≈568.67, fy≈568.37） |
| depth CameraInfo P | 等于彩色 P（标定值） |
| `camera_link -> camera_color_optical_frame` | 平移 0，旋转 -90°（光学坐标系） |
| `camera_depth_optical_frame -> camera_color_optical_frame` | 单位变换（硬件已配准） |

> TF 中的单位变换表示“深度图已经被硬件配准到彩色”，不是两传感器物理零基线。

---

## 7. RGB-D 同步

使用 `message_filters::ApproximateTime`，`approx_slop_s: 0.05`。

实测同步统计（`/perception/sync_stats`）：

| 指标 | 值 |
|---|---|
| `time_diff_ms`（color - depth） | 约 -17ms 到 +6ms，单次均值 -8.7ms |
| `color_rate_hz` | 30.17 |
| `depth_rate_hz` | 29.81 |
| `gap_count` | 0（5 秒采样窗口） |
| `valid_depth_ratio` | 58%–80%（场景相关） |
| `point_count` | 约 4.4 万–6.1 万（step=2） |
| `process_ms` | 约 2.4–6.8ms |

这不是硬同步；30Hz 下 10–17ms 的偏差可接受，检测框和深度使用近似时间配对。

---

## 8. 深度有效性

- 冷启动前若干帧可能全 0；连续订阅约 1 秒后稳定。
- 深色/反光/透明物体、显示器屏幕、超出 8m 的区域可能无效。
- 实测办公室场景：全图有效率约 58%–80%；检测 ROI 内成功率约 85%。
- 本项目对 0、非有限值、<0.3m、>8m 的深度做过滤，不用单像素中心值。

---

## 9. 验收结论

| 验收项 | 结果 |
|---|---|
| RGB 持续输出 | ✅ 30Hz，rgb8 |
| Depth 持续输出 | ✅ 29.8Hz，16UC1 |
| CameraInfo 可读 | ✅ 彩色/深度 K 均为标定值 |
| 时间戳正常 | ✅ ROS epoch，约 ±17ms 配对 |
| USB 稳定 | ✅ 测试期间无断连 |
| rqt_image_view/rviz 可看 | ✅ topic 正常；RViz 配置已提供 |
| 深度单位 | ✅ 毫米，scale=0.001 |
| 深度对齐 | ✅ depth_align=true 可用 |
| 真实彩色内参 | ✅ 已标定，见第 5 节 |
| 0.8–2.0m 精度 | ⚠️ 待用户配合实测 |

---

## 10. 复核命令

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash

rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_raw
rostopic echo -n1 --noarr /camera/color/image_raw
rostopic echo -n1 --noarr /camera/depth/image_raw
rostopic echo -n1 /camera/color/camera_info
rostopic echo -n1 /camera/depth/camera_info

lsusb -nn | grep 2bc5
ldd ~/astra_rgbd_ws/devel/lib/astra_camera/astra_camera_node | grep -i openni
```

标定后确认：

```bash
rostopic echo -n1 /camera/color/camera_info | head -20
# K[0]/K[4] 应非 0
```