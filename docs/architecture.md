# 系统架构

## 1. 总体数据流

```
                 Orbbec Astra Pro (USB 2.0)
                 |                         |
        UVC RGB (2bc5:0501)      OpenNI2 Depth (2bc5:0403)
                 |                         |
                 +-------- astra_camera ---+
                     depth_align:=true
                     /camera/color/image_raw       rgb8   640x480
                     /camera/depth/image_raw       16UC1  640x480, mm
                     /camera/color/camera_info
                     /camera/depth/camera_info
                     /tf
                              |
              +---------------+----------------+
              |                                |
      rgbd_sync_node                     detector_node
      - ApproximateTime sync             - letterbox 640x640
      - XYZ / XYZRGB PCL clouds          - TensorRT FP16 YOLOv5n
      - sync statistics                  - decode + class-wise NMS
              |                                |
   /perception/points_xyz                  /perception/detections_2d
   /perception/points_xyzrgb               /perception/image_annotated
   /perception/sync_stats                         |
                                                  v
                                        depth_localizer_node
                                        - central 50% ROI
                                        - median depth / valid ratio
                                        - back-projection with CameraInfo
                                                  |
                                        /perception/objects_3d
                                        /perception/markers
                                                  |
                                                 RViz
```

## 2. ROS 节点

| 节点 | 可执行文件 | 作用 | 关键输入 | 关键输出 |
|---|---|---|---|---|
| `camera` | `astra_camera_node` | Astra Pro 驱动，发布 RGB/Depth/CameraInfo/TF | USB | `/camera/*` |
| `rgbd_sync` | `rgbd_sync_node` | RGB-D 同步、点云、统计 | color/depth image + CameraInfo | `points_xyz`, `points_xyzrgb`, `sync_stats` |
| `detector` | `detector_node` | TensorRT YOLOv5n 检测 | color image | `detections_2d`, `image_annotated` |
| `depth_localizer` | `depth_localizer_node` | 深度 ROI + 3D 反投影 + RViz 标记 | detections + depth + CameraInfo | `objects_3d`, `markers` |
| `rviz` | `rviz` | 可视化 | 图像/点云/markers | 屏幕 |

`system.launch` 负责按顺序启动以上节点；各节点也可以单独启动。

## 3. 消息设计

### `Detection2D.msg`

- `class_id` / `class_name` / `confidence`
- `xmin/ymin/xmax/ymax`：彩色图像像素坐标（闭区间）

### `Detection2DArray.msg`

- 图像尺寸
- `preprocess_ms` / `inference_ms` / `postprocess_ms`
- `Detection2D[] detections`

### `Object3D.msg`

- `class_id` / `class_name` / `confidence`
- 2D 框
- `depth_m`：ROI 有效深度中位数（米）
- `valid_depth_pixels` / `valid_depth_ratio`
- `position`：相机光学坐标系下的 X/Y/Z（米）
- `depth_valid`：有效深度不足时为 `false`，此时 `position` 无意义

### `Object3DArray.msg`

- `Header`（时间戳与坐标系）
- `Object3D[] objects`

### `RgbdSyncStats.msg`

- `time_diff_ms`：color stamp - depth stamp
- `color_rate_hz` / `depth_rate_hz`
- `valid_depth_ratio`：采样深度中落在有效范围内的比例
- `point_count`：最新点云点数
- `process_ms`：点云构建耗时
- `gap_count`：同步回调间隔超过阈值的次数
- `using_fallback_intrinsics`：是否使用了临时内参

## 4. 坐标系与三维计算

### 坐标系

- 彩色光学坐标系：`camera_color_optical_frame`
  - X 向右
  - Y 向下
  - Z 向前
- 彩色相机本体：`camera_color_frame`
- 深度光学坐标系：`camera_depth_optical_frame`
- 当 `depth_align:=true` 时，驱动输出的深度图已经在
  `camera_color_optical_frame` 下，深度 `CameraInfo` 也使用彩色内参。
- 未对齐时深度图在 `camera_depth_optical_frame` 下，本项目默认
  `require_aligned: true`，不会在未对齐时输出 RGB 对应的 XYZ。

### 深度单位

Astra Pro 深度为 `16UC1`，单位为毫米。本项目参数：

```yaml
depth_scale_to_m: 0.001
Z = raw_depth * 0.001   # meters
```

不要在 `16UC1` 数据上直接使用 `depth_scale: 1`；也不要假设毫米/米可以混用。

### ROI 与反投影

对每个 2D 检测框：

1. 取检测框中心 50%（`roi_ratio: 0.5`）。
2. 遍历 ROI 内深度像素：
   - 跳过 `0`、非有限值；
   - 过滤 `< 0.3m` 或 `> 8.0m` 的异常值。
3. 有效像素数 `< min_valid_pixels`（10）或有效率 `< min_valid_ratio`（0.05）时，
   标记 `depth_valid=false`，不输出虚假坐标。
4. 对有效深度取中位数作为 `Z`。
5. 使用有效像素的平均 `(u, v)` 作为反投影点：

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = median(depth)
```

内参来自 `CameraInfo`：

- 对齐深度：`/camera/depth/camera_info` 的 K（本项目补丁使其等于彩色 K）。
- 若 K 无效且 `fallback_intrinsics: true`，使用临时值
  `fx=fy=570.3422, cx=319.5, cy=239.5`，日志会打印警告。

> 本机已于 2026-11-22 用 `scripts/calibrate_rgb.py` 完成彩色标定，
> `fallback_intrinsics` 已关闭；彩色与深度 CameraInfo 使用同一组真实 K。
> 换相机、改分辨率或重新打印棋盘格后需要重新标定。

## 5. 检测后处理

YOLOv5n ONNX 输出 `output0`，形状 `1x25200x85`：

- 前 4 个值：`cx, cy, w, h`（640×640 letterbox 像素坐标）
- 第 5 个值：objectness
- 后 80 个值：COCO 类别分数

流程：

1. 过滤 `objectness < conf_threshold`。
2. 取最大类别分数，`score = objectness * class_score`。
3. 反 letterbox 到原始 640×480。
4. 按类别做 IoU NMS（`iou_threshold`），最多 `max_detections` 个。
5. 发布 `Detection2DArray` 和绘制框后的 `image_annotated`。

类别名来自 `config/coco.names`（80 类）。

## 6. TensorRT 封装

`TrtEngine`（`include/astra_rgbd_perception/trt_engine.hpp`）：

- 反序列化 engine，查找唯一输入/输出 binding。
- 根据 binding dtype 分配 FP32/FP16 设备缓冲区。
- `infer(const float* input, ...)`：
  - FP16 binding 时在主机端做 `float -> __half`；
  - `cudaMemcpyAsync` H2D；
  - `enqueueV2`；
  - D2H；
  - FP16 输出时做 `__half -> float`。
- 当前 engine 的输入/输出均为 FP16 binding（`dtype=1`）。

engine 只适用于构建它的设备/TensorRT 版本；换板或升级 TensorRT 后必须重新构建。

## 7. 线程与性能

- 每个 ROS 节点单线程回调；检测节点一次处理一帧，订阅队列为 1，避免积压。
- 点云节点按 `cloud_rate_hz` 限速（默认 10Hz），避免 30Hz 全量点云占满带宽。
- 检测节点 CMake 显式 `-O3`；否则主机端 FP16 转换和预处理会慢约 4 倍。
- `rosbag play --clock` 可驱动无相机离线推理。

## 8. 代码结构

```
src/astra_rgbd_perception/
├── CMakeLists.txt
├── package.xml
├── include/astra_rgbd_perception/trt_engine.hpp
├── src/
│   ├── rgbd_sync_node.cpp
│   ├── detector_node.cpp
│   ├── depth_localizer_node.cpp
│   └── trt_engine.cpp
├── msg/
│   ├── Detection2D.msg
│   ├── Detection2DArray.msg
│   ├── Object3D.msg
│   ├── Object3DArray.msg
│   └── RgbdSyncStats.msg
├── launch/
│   ├── rgbd.launch
│   ├── detector.launch
│   ├── localizer.launch
│   └── system.launch
├── config/
│   ├── rgbd_sync.yaml
│   ├── detector.yaml
│   ├── localizer.yaml
│   ├── coco.names
│   └── rgbd_view.rviz
└── scripts/
    ├── record_rgbd.sh
    ├── play_rgbd.sh
    ├── benchmark.sh
    ├── stability_test.sh
    └── calibrate_rgb.py
```