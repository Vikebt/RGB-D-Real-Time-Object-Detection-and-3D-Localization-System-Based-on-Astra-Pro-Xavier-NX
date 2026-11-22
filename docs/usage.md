# 使用说明（命令速查）

完整流程见 `docs/user_manual.md`；本文是日常操作和参数速查。

---

## 1. 环境

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash
```

---

## 2. 启动

### 完整系统

```bash
roslaunch astra_rgbd_perception system.launch                 # 含 RViz
roslaunch astra_rgbd_perception system.launch use_rviz:=false # 无显示器/SSH
```

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `camera_name` | `camera` | 相机命名空间 |
| `start_camera` | `true` | 是否启动 Astra 驱动 |
| `depth_align` | `true` | 深度对齐到彩色 |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_sync` | `true` | 点云/同步节点 |
| `use_detector` | `true` | YOLO 检测 |
| `use_localizer` | `true` | 三维定位 |
| `engine_path` | `.../models/yolov5n_fp16.engine` | TensorRT engine |

### 分模块

```bash
# 相机（深度对齐、关闭驱动自带点云）
roslaunch astra_camera astra_pro.launch \
  depth_align:=true enable_point_cloud:=false enable_ir:=false

# RGB-D 同步/点云（复用已启动的相机）
roslaunch astra_rgbd_perception rgbd.launch start_camera:=false

# 检测
roslaunch astra_rgbd_perception detector.launch

# 三维定位
roslaunch astra_rgbd_perception localizer.launch

# 无相机离线回放时
roslaunch astra_rgbd_perception system.launch start_camera:=false use_rviz:=false
```

---

## 3. Topic

| Topic | 类型 | 方向 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 相机 → 检测 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 相机 → 同步/定位 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 相机（对齐后为彩色内参） |
| `/perception/detections_2d` | `astra_rgbd_perception/Detection2DArray` | 检测输出 |
| `/perception/image_annotated` | `sensor_msgs/Image` | 检测标注图（bgr8） |
| `/perception/objects_3d` | `astra_rgbd_perception/Object3DArray` | 三维定位输出 |
| `/perception/markers` | `visualization_msgs/MarkerArray` | RViz 标记 |
| `/perception/points_xyz` | `sensor_msgs/PointCloud2` | XYZ 点云 |
| `/perception/points_xyzrgb` | `sensor_msgs/PointCloud2` | XYZRGB 点云 |
| `/perception/sync_stats` | `astra_rgbd_perception/RgbdSyncStats` | 同步/点云统计 |

---

## 4. 配置参数

### `config/rgbd_sync.yaml`

| 参数 | 默认 | 说明 |
|---|---|---|
| `step` | 2 | 点云采样步长 |
| `min_range_m` / `max_range_m` | 0.3 / 8.0 | 深度有效范围 |
| `depth_scale_to_m` | 0.001 | `16UC1` 毫米→米 |
| `cloud_rate_hz` | 10.0 | 点云发布频率 |
| `publish_xyz` / `publish_xyzrgb` | true | 点云类型 |
| `require_aligned` | true | 深度必须在彩色坐标系 |
| `fallback_intrinsics` | true（临时） | K 无效时使用临时内参 |
| `approx_slop_s` | 0.05 | 同步最大时间差 |

### `config/detector.yaml`

| 参数 | 默认 | 说明 |
|---|---|---|
| `conf_threshold` | 0.25 | 置信度阈值 |
| `iou_threshold` | 0.45 | NMS IoU |
| `max_detections` | 100 | 最大检测数 |
| `input_size` | 640 | 必须与 engine 匹配 |
| `publish_annotated` | true | 发布标注图 |
| `debug_output` | false | 打印原始 YOLO 输出统计（诊断） |

### `config/localizer.yaml`

| 参数 | 默认 | 说明 |
|---|---|---|
| `roi_ratio` | 0.5 | 检测框中心 ROI 比例 |
| `min_valid_pixels` | 10 | 最少有效深度像素 |
| `min_valid_ratio` | 0.05 | 最低有效比例 |
| `depth_scale_to_m` | 0.001 | 毫米→米 |
| `min_range_m` / `max_range_m` | 0.3 / 8.0 | 深度范围 |
| `marker_lifetime_s` | 0.5 | RViz marker 生命周期 |

---

## 5. 查看

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_raw
rostopic hz /perception/detections_2d
rostopic hz /perception/objects_3d
rostopic hz /perception/points_xyzrgb

rostopic echo -n1 /perception/sync_stats
rostopic echo -n1 /perception/detections_2d
rostopic echo -n1 /perception/objects_3d

rqt_image_view /perception/image_annotated
rviz -d ~/astra_rgbd_ws/src/astra_rgbd_perception/config/rgbd_view.rviz
```

`objects_3d` 字段：

- `class_name` / `class_id` / `confidence`
- `xmin/ymin/xmax/ymax`
- `depth_m`
- `valid_depth_pixels` / `valid_depth_ratio`
- `position.x/y/z`（米，相机彩色光学坐标系）
- `depth_valid`

`depth_valid=false` 时 `position` 不可用。

---

## 6. 录制、回放、性能

```bash
# 录制 10 秒（不传参则直到 Ctrl-C）
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/record_rgbd.sh 10

# 回放
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/play_rgbd.sh \
  ~/astra_rgbd_ws/bags/rgbd_20261121_195513.bag -r 1

# 性能采样
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/benchmark.sh 10

# 10 分钟稳定性
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/stability_test.sh 600 30

# 三维距离实测（把物体放在卷尺量定的距离）
python2 ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/measure_distance.py \
  --class bottle --duration 5 --show-all
```

日志与 bag 默认在：

- `~/astra_rgbd_ws/logs/`
- `~/astra_rgbd_ws/bags/`

---

## 7. 标定

```bash
# 终端 1：相机
roslaunch astra_camera astra_pro.launch depth_align:=true \
  enable_point_cloud:=false enable_ir:=false

# 终端 2：自动标定
python2 ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/calibrate_rgb.py \
  --cols 9 --rows 6 --square 0.025 --min-views 15 --max-views 25 --timeout 300
```

标定文件：`~/.ros/camera_info/rgb_camera.yaml`。

---

## 8. 停止

- 前台 `roslaunch`：`Ctrl-C`。
- 后台：

```bash
pkill -f astra_camera_node
pkill -f rgbd_sync_node
pkill -f detector_node
pkill -f depth_localizer_node
pkill -f roslaunch
```

不要使用 `pkill -9`，让节点有机会释放相机和 CUDA 资源。